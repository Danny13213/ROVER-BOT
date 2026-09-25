#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;


// ============================================================
// MODES
// ============================================================

enum Mode
{
    MANUAL,
    AUTO
};

enum AutoState
{
    AUTO_DRIVE,
    AUTO_STOPPED,
    AUTO_SCAN,
    AUTO_TURN_TO_BEST,
    AUTO_REVERSE
};


// ============================================================
// NAVIGATION DATA
// ============================================================

struct NavData
{
    double timestamp;

    double left;
    double center;
    double right;

    double throttle;
    double steer;

    string status;
};


// ============================================================
// SERIAL
// ============================================================

void sendCommand(
    int esp,
    char command
)
{
    write(
        esp,
        &command,
        1
    );
}


// ============================================================
// MOTOR PWM
// ============================================================

void setLeftSpeed(
    int esp,
    int speed
)
{
    speed =
        max(
            0,
            min(255, speed)
        );

    string command =
        "A"
        + to_string(speed)
        + "\n";

    write(
        esp,
        command.c_str(),
        command.length()
    );
}


void setRightSpeed(
    int esp,
    int speed
)
{
    speed =
        max(
            0,
            min(255, speed)
        );

    string command =
        "C"
        + to_string(speed)
        + "\n";

    write(
        esp,
        command.c_str(),
        command.length()
    );
}


// ============================================================
// SERIAL SETUP
// ============================================================

bool setupSerial(int fd)
{
    termios tty{};

    if (tcgetattr(fd, &tty) != 0)
        return false;

    cfsetospeed(
        &tty,
        B115200
    );

    cfsetispeed(
        &tty,
        B115200
    );

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_iflag &=
        ~(IXON | IXOFF | IXANY);

    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cflag |=
        (CLOCAL | CREAD);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    return (
        tcsetattr(
            fd,
            TCSANOW,
            &tty
        ) == 0
    );
}


// ============================================================
// READ CAMERA NAVIGATION
// ============================================================

bool readNavigation(
    NavData &nav
)
{
    ifstream file(
        "/tmp/rover_nav.txt"
    );

    if (!file.is_open())
        return false;

    file
        >> nav.timestamp
        >> nav.left
        >> nav.center
        >> nav.right
        >> nav.throttle
        >> nav.steer
        >> nav.status;

    return !file.fail();
}


// ============================================================
// CAMERA WATCHDOG
// ============================================================

bool navigationFresh(
    const NavData &nav
)
{
    auto now =
        system_clock::now();

    double currentTime =
        duration<double>(
            now.time_since_epoch()
        ).count();

    double age =
        currentTime
        - nav.timestamp;

    return (
        age >= 0.0
        &&
        age <= 2.0
    );
}


// ============================================================
// AUTONOMOUS PWM MIXER
// ============================================================

void calculateAutoPWM(
    double throttle,
    double steer,
    int maxLeft,
    int maxRight,
    int &leftPWM,
    int &rightPWM
)
{
    throttle =
        max(
            0.0,
            min(
                1.0,
                fabs(throttle)
            )
        );

    steer =
        max(
            -1.0,
            min(
                1.0,
                steer
            )
        );

    double leftFactor =
        throttle;

    double rightFactor =
        throttle;


    // LEFT
    if (steer < 0.0)
    {
        leftFactor *=
            (
                1.0
                - fabs(steer)
            );
    }

    // RIGHT
    else if (steer > 0.0)
    {
        rightFactor *=
            (
                1.0
                - fabs(steer)
            );
    }


    leftPWM =
        static_cast<int>(
            maxLeft
            * leftFactor
        );

    rightPWM =
        static_cast<int>(
            maxRight
            * rightFactor
        );


    leftPWM =
        max(
            0,
            min(
                255,
                leftPWM
            )
        );

    rightPWM =
        max(
            0,
            min(
                255,
                rightPWM
            )
        );
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    const char* espDevice =
        "/dev/ttyUSB0";


    int esp =
        open(
            espDevice,
            O_RDWR | O_NOCTTY
        );


    if (esp < 0)
    {
        cerr
            << "ERROR: Could not open "
            << espDevice
            << "\n";

        return 1;
    }


    if (!setupSerial(esp))
    {
        cerr
            << "ERROR: Serial setup failed.\n";

        close(esp);

        return 1;
    }


    // ========================================================
    // TERMINAL
    // ========================================================

    termios oldTerminal;
    termios newTerminal;

    tcgetattr(
        STDIN_FILENO,
        &oldTerminal
    );

    newTerminal =
        oldTerminal;

    newTerminal.c_lflag &=
        ~(ICANON | ECHO);

    newTerminal.c_cc[VMIN] = 0;
    newTerminal.c_cc[VTIME] = 0;

    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &newTerminal
    );


    int oldFlags =
        fcntl(
            STDIN_FILENO,
            F_GETFL,
            0
        );

    fcntl(
        STDIN_FILENO,
        F_SETFL,
        oldFlags | O_NONBLOCK
    );


    // ========================================================
    // SETTINGS
    // ========================================================

    Mode mode =
        MANUAL;

    AutoState autoState =
        AUTO_DRIVE;


    int leftSpeed =
        200;

    int rightSpeed =
        200;

    const int SPEED_STEP =
        5;


    // --------------------------------------------------------
    // SCAN SETTINGS
    // --------------------------------------------------------

    const int SCAN_PWM =
        110;

    /*
     * IMPORTANT:
     *
     * This must be calibrated.
     *
     * Start around 4 seconds.
     *
     * Increase if rover turns < 360 degrees.
     * Decrease if rover turns > 360 degrees.
     */

    const milliseconds
        FULL_SCAN_TIME(4000);


    // Number of samples during 360 scan

    const int SCAN_SECTIONS =
        16;


    // Require this much center clearance
    // to consider a direction usable.

    const double
        MIN_GOOD_CLEARANCE = 1.50;


    // --------------------------------------------------------
    // REVERSE SETTINGS
    // --------------------------------------------------------

    const int REVERSE_PWM =
        70;

    const milliseconds
        REVERSE_TIME(700);


    // --------------------------------------------------------
    // STOP BEFORE SCAN
    // --------------------------------------------------------

    const milliseconds
        STUCK_TIME(800);


    // --------------------------------------------------------
    // MANUAL DEAD MAN
    // --------------------------------------------------------

    bool moving =
        false;

    char currentMovement =
        'S';

    auto lastMovementCommand =
        steady_clock::now();

    const milliseconds
        STOP_TIMEOUT(220);


    // --------------------------------------------------------
    // AUTO TIMERS
    // --------------------------------------------------------

    auto lastAutoUpdate =
        steady_clock::now();

    const milliseconds
        AUTO_UPDATE_INTERVAL(100);


    auto stopStart =
        steady_clock::now();

    auto scanStart =
        steady_clock::now();

    auto turnStart =
        steady_clock::now();

    auto reverseStart =
        steady_clock::now();


    // ========================================================
    // SCAN VARIABLES
    // ========================================================

    double bestClearance =
        0.0;

    int bestSection =
        0;

    int lastScanSection =
        -1;


    // ========================================================
    // INITIAL STOP
    // ========================================================

    sendCommand(
        esp,
        'S'
    );

    setLeftSpeed(
        esp,
        leftSpeed
    );

    this_thread::sleep_for(
        milliseconds(50)
    );

    setRightSpeed(
        esp,
        rightSpeed
    );


    // ========================================================
    // CONTROLS
    // ========================================================

    cout << "\n";
    cout << "=============================\n";
    cout << "        ROVER CONTROL\n";
    cout << "=============================\n\n";

    cout << "W = Forward\n";
    cout << "S = Reverse\n";
    cout << "A = Left\n";
    cout << "D = Right\n\n";

    cout << "U = Left PWM +5\n";
    cout << "J = Left PWM -5\n";
    cout << "I = Right PWM +5\n";
    cout << "K = Right PWM -5\n";
    cout << "P = Show PWM\n\n";

    cout << "M     = Manual / Auto\n";
    cout << "SPACE = STOP\n";
    cout << "Q     = Quit\n\n";


    // ========================================================
    // MAIN LOOP
    // ========================================================

    bool running =
        true;


    while (running)
    {
        char key;


        ssize_t bytes =
            read(
                STDIN_FILENO,
                &key,
                1
            );


        // ====================================================
        // KEYBOARD
        // ====================================================

        if (bytes > 0)
        {
            if (
                key >= 'A'
                &&
                key <= 'Z'
            )
            {
                key =
                    key
                    - 'A'
                    + 'a';
            }


            // ------------------------------------------------
            // STOP
            // ------------------------------------------------

            if (key == ' ')
            {
                sendCommand(
                    esp,
                    'S'
                );

                moving = false;

                autoState =
                    AUTO_DRIVE;

                cout
                    << "\n*** STOP ***\n";
            }


            // ------------------------------------------------
            // QUIT
            // ------------------------------------------------

            else if (key == 'q')
            {
                sendCommand(
                    esp,
                    'S'
                );

                running =
                    false;
            }


            // ------------------------------------------------
            // MODE
            // ------------------------------------------------

            else if (key == 'm')
            {
                sendCommand(
                    esp,
                    'S'
                );

                moving =
                    false;

                autoState =
                    AUTO_DRIVE;


                if (mode == MANUAL)
                {
                    mode =
                        AUTO;

                    cout
                        << "\nAUTO MODE\n";
                }

                else
                {
                    mode =
                        MANUAL;

                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );

                    setRightSpeed(
                        esp,
                        rightSpeed
                    );

                    cout
                        << "\nMANUAL MODE\n";
                }
            }


            // ------------------------------------------------
            // PWM CONTROLS
            // ------------------------------------------------

            else if (key == 'u')
            {
                leftSpeed =
                    min(
                        255,
                        leftSpeed + SPEED_STEP
                    );

                if (mode == MANUAL)
                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );

                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            else if (key == 'j')
            {
                leftSpeed =
                    max(
                        0,
                        leftSpeed - SPEED_STEP
                    );

                if (mode == MANUAL)
                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );

                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            else if (key == 'i')
            {
                rightSpeed =
                    min(
                        255,
                        rightSpeed + SPEED_STEP
                    );

                if (mode == MANUAL)
                    setRightSpeed(
                        esp,
                        rightSpeed
                    );

                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            else if (key == 'k')
            {
                rightSpeed =
                    max(
                        0,
                        rightSpeed - SPEED_STEP
                    );

                if (mode == MANUAL)
                    setRightSpeed(
                        esp,
                        rightSpeed
                    );

                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            else if (key == 'p')
            {
                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << " RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            // =================================================
            // MANUAL WASD
            // =================================================

            else if (mode == MANUAL)
            {
                char command =
                    0;


                if (key == 'w')
                {
                    command = 'F';
                }

                else if (key == 's')
                {
                    command = 'B';
                }

                // Your physical steering is reversed
                // relative to the ESP commands.

                else if (key == 'a')
                {
                    command = 'R';
                }

                else if (key == 'd')
                {
                    command = 'L';
                }


                if (command != 0)
                {
                    if (
                        !moving
                        ||
                        command
                        != currentMovement
                    )
                    {
                        sendCommand(
                            esp,
                            command
                        );

                        currentMovement =
                            command;

                        moving =
                            true;
                    }


                    lastMovementCommand =
                        steady_clock::now();
                }
            }
        }


        // ====================================================
        // MANUAL DEAD-MAN
        // ====================================================

        if (
            mode == MANUAL
            &&
            moving
        )
        {
            auto now =
                steady_clock::now();


            if (
                duration_cast<milliseconds>(
                    now
                    - lastMovementCommand
                )
                >
                STOP_TIMEOUT
            )
            {
                sendCommand(
                    esp,
                    'S'
                );

                moving =
                    false;

                currentMovement =
                    'S';
            }
        }


        // ====================================================
        // AUTONOMOUS
        // ====================================================

        if (mode == AUTO)
        {
            auto now =
                steady_clock::now();


            // =================================================
            // NORMAL DRIVE
            // =================================================

            if (
                autoState
                == AUTO_DRIVE
            )
            {
                if (
                    duration_cast<milliseconds>(
                        now
                        - lastAutoUpdate
                    )
                    >=
                    AUTO_UPDATE_INTERVAL
                )
                {
                    lastAutoUpdate =
                        now;


                    NavData nav;


                    // -----------------------------------------
                    // CAMERA FAILURE
                    // -----------------------------------------

                    if (
                        !readNavigation(nav)
                        ||
                        !navigationFresh(nav)
                    )
                    {
                        sendCommand(
                            esp,
                            'S'
                        );

                        cout
                            << "AUTO: CAMERA ERROR -> STOP\n";

                        continue;
                    }


                    // -----------------------------------------
                    // BLOCKED
                    // -----------------------------------------

                    if (
                        nav.status == "STOP"
                        ||
                        nav.throttle <= 0.0
                    )
                    {
                        sendCommand(
                            esp,
                            'S'
                        );

                        autoState =
                            AUTO_STOPPED;

                        stopStart =
                            now;

                        cout
                            << "AUTO: BLOCKED\n";

                        continue;
                    }


                    // -----------------------------------------
                    // NORMAL MOVEMENT
                    // -----------------------------------------

                    int autoLeft;
                    int autoRight;


                    calculateAutoPWM(
                        nav.throttle,
                        nav.steer,
                        leftSpeed,
                        rightSpeed,
                        autoLeft,
                        autoRight
                    );


                    setLeftSpeed(
                        esp,
                        autoLeft
                    );

                    setRightSpeed(
                        esp,
                        autoRight
                    );

                    sendCommand(
                        esp,
                        'F'
                    );


                    cout
                        << "AUTO: "
                        << nav.status
                        << " C="
                        << nav.center
                        << "m"
                        << " steer="
                        << nav.steer
                        << " PWM="
                        << autoLeft
                        << "/"
                        << autoRight
                        << "\n";
                }
            }


            // =================================================
            // WAIT BEFORE SCANNING
            // =================================================

            else if (
                autoState
                == AUTO_STOPPED
            )
            {
                sendCommand(
                    esp,
                    'S'
                );


                if (
                    duration_cast<milliseconds>(
                        now
                        - stopStart
                    )
                    >=
                    STUCK_TIME
                )
                {
                    cout
                        << "\nSTARTING 360 SCAN\n";


                    bestClearance =
                        0.0;

                    bestSection =
                        0;

                    lastScanSection =
                        -1;


                    scanStart =
                        now;


                    setLeftSpeed(
                        esp,
                        SCAN_PWM
                    );

                    setRightSpeed(
                        esp,
                        SCAN_PWM
                    );


                    /*
                     * Your ESP32 R command performs
                     * an in-place physical turn.
                     *
                     * If this rotates the wrong way,
                     * change 'R' to 'L'.
                     */

                    sendCommand(
                        esp,
                        'R'
                    );


                    autoState =
                        AUTO_SCAN;
                }
            }


            // =================================================
            // 360 SCAN
            // =================================================

            else if (
                autoState
                == AUTO_SCAN
            )
            {
                auto elapsed =
                    duration_cast<milliseconds>(
                        now
                        - scanStart
                    );


                double progress =
                    static_cast<double>(
                        elapsed.count()
                    )
                    /
                    FULL_SCAN_TIME.count();


                int section =
                    static_cast<int>(
                        progress
                        * SCAN_SECTIONS
                    );


                section =
                    max(
                        0,
                        min(
                            SCAN_SECTIONS - 1,
                            section
                        )
                    );


                // ---------------------------------------------
                // SAMPLE EACH SECTION
                // ---------------------------------------------

                if (
                    section
                    != lastScanSection
                )
                {
                    lastScanSection =
                        section;


                    NavData nav;


                    if (
                        readNavigation(nav)
                        &&
                        navigationFresh(nav)
                    )
                    {
                        /*
                         * Use CENTER depth because the
                         * camera is pointing directly
                         * at the direction currently
                         * being scanned.
                         */

                        double clearance =
                            nav.center;


                        if (
                            !isinf(clearance)
                            &&
                            clearance
                            >
                            bestClearance
                        )
                        {
                            bestClearance =
                                clearance;

                            bestSection =
                                section;
                        }


                        cout
                            << "SCAN "
                            << section
                            << "/"
                            << SCAN_SECTIONS
                            << " center="
                            << clearance
                            << "m\n";
                    }
                }


                // ---------------------------------------------
                // FINISHED 360
                // ---------------------------------------------

                if (
                    elapsed
                    >=
                    FULL_SCAN_TIME
                )
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "\nSCAN COMPLETE\n";

                    cout
                        << "Best clearance: "
                        << bestClearance
                        << "m\n";


                    // -----------------------------------------
                    // NO GOOD PATH
                    // -----------------------------------------

                    if (
                        bestClearance
                        <
                        MIN_GOOD_CLEARANCE
                    )
                    {
                        cout
                            << "No safe route."
                            << " Short reverse.\n";


                        setLeftSpeed(
                            esp,
                            REVERSE_PWM
                        );

                        setRightSpeed(
                            esp,
                            REVERSE_PWM
                        );


                        sendCommand(
                            esp,
                            'B'
                        );


                        reverseStart =
                            now;


                        autoState =
                            AUTO_REVERSE;
                    }


                    // -----------------------------------------
                    // TURN TOWARD BEST DIRECTION
                    // -----------------------------------------

                    else
                    {
                        /*
                         * Each section represents:
                         *
                         * 360 / 16 = 22.5 degrees
                         */

                        double fraction =
                            static_cast<double>(
                                bestSection
                            )
                            /
                            SCAN_SECTIONS;


                        auto turnDuration =
                            milliseconds(
                                static_cast<long>(
                                    FULL_SCAN_TIME.count()
                                    *
                                    fraction
                                )
                            );


                        cout
                            << "Best scan section: "
                            << bestSection
                            << "\n";

                        cout
                            << "Turning toward opening...\n";


                        setLeftSpeed(
                            esp,
                            SCAN_PWM
                        );

                        setRightSpeed(
                            esp,
                            SCAN_PWM
                        );


                        sendCommand(
                            esp,
                            'R'
                        );


                        turnStart =
                            now;


                        /*
                         * Reuse scanStart to store
                         * the required turn duration.
                         */

                        scanStart =
                            steady_clock::time_point(
                                milliseconds(
                                    turnDuration.count()
                                )
                            );


                        autoState =
                            AUTO_TURN_TO_BEST;
                    }
                }
            }


            // =================================================
            // TURN BACK TO BEST SCAN DIRECTION
            // =================================================

            else if (
                autoState
                == AUTO_TURN_TO_BEST
            )
            {
                /*
                 * scanStart is being used here to
                 * store a duration in milliseconds.
                 */

                long requiredMs =
                    duration_cast<milliseconds>(
                        scanStart.time_since_epoch()
                    ).count();


                long elapsedMs =
                    duration_cast<milliseconds>(
                        now
                        - turnStart
                    ).count();


                if (
                    elapsedMs
                    >=
                    requiredMs
                )
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "OPENING SELECTED\n";


                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );

                    setRightSpeed(
                        esp,
                        rightSpeed
                    );


                    autoState =
                        AUTO_DRIVE;


                    lastAutoUpdate =
                        now;
                }
            }


            // =================================================
            // SHORT REVERSE RECOVERY
            // =================================================

            else if (
                autoState
                == AUTO_REVERSE
            )
            {
                if (
                    duration_cast<milliseconds>(
                        now
                        - reverseStart
                    )
                    >=
                    REVERSE_TIME
                )
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "REVERSE COMPLETE\n";


                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );

                    setRightSpeed(
                        esp,
                        rightSpeed
                    );


                    /*
                     * Return to normal navigation.
                     * The front camera will immediately
                     * re-evaluate the situation.
                     */

                    autoState =
                        AUTO_DRIVE;


                    lastAutoUpdate =
                        now;
                }
            }
        }


        this_thread::sleep_for(
            milliseconds(5)
        );
    }


    // ========================================================
    // SHUTDOWN
    // ========================================================

    sendCommand(
        esp,
        'S'
    );


    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &oldTerminal
    );


    fcntl(
        STDIN_FILENO,
        F_SETFL,
        oldFlags
    );


    close(esp);


    cout
        << "\nController closed.\n";


    return 0;
}
