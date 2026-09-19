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
// MODE
// ============================================================

enum Mode
{
    MANUAL,
    AUTO
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
// SERIAL COMMAND
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
// LEFT MOTOR PWM
// ============================================================

void setLeftSpeed(
    int esp,
    int speed
)
{
    speed = max(
        0,
        min(
            255,
            speed
        )
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


// ============================================================
// RIGHT MOTOR PWM
// ============================================================

void setRightSpeed(
    int esp,
    int speed
)
{
    speed = max(
        0,
        min(
            255,
            speed
        )
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

bool setupSerial(
    int fd
)
{
    termios tty{};

    if (
        tcgetattr(
            fd,
            &tty
        ) != 0
    )
    {
        return false;
    }


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
        ~(
            IXON
            | IXOFF
            | IXANY
        );


    tty.c_lflag = 0;

    tty.c_oflag = 0;


    tty.c_cflag |=
        (
            CLOCAL
            | CREAD
        );


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
// READ NAVIGATION FILE
// ============================================================

bool readNavigation(
    NavData &nav
)
{
    ifstream file(
        "/tmp/rover_nav.txt"
    );

    if (!file.is_open())
    {
        return false;
    }


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
// CHECK CAMERA DATA AGE
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


    // Camera information older than
    // 500 ms is considered unsafe.

    return (
        age >= 0.0
        && age <= 0.5
    );
}


// ============================================================
// AUTONOMOUS MOTOR MIXING
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
    // --------------------------------------------------------
    // NO AUTONOMOUS REVERSE
    // --------------------------------------------------------

    if (throttle <= 0.0)
    {
        leftPWM = 0;
        rightPWM = 0;

        return;
    }


    throttle = max(
        0.0,
        min(
            1.0,
            throttle
        )
    );


    steer = max(
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


    // --------------------------------------------------------
    // LEFT TURN
    //
    // Negative steer = left.
    // Slow LEFT track.
    // --------------------------------------------------------

    if (steer < 0.0)
    {
        leftFactor *=
            (
                1.0
                - fabs(steer)
            );
    }


    // --------------------------------------------------------
    // RIGHT TURN
    //
    // Positive steer = right.
    // Slow RIGHT track.
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // OPEN ESP32
    // --------------------------------------------------------

    int esp = open(
        espDevice,
        O_RDWR
        | O_NOCTTY
    );


    if (esp < 0)
    {
        cerr
            << "ERROR: Could not open "
            << espDevice
            << endl;

        return 1;
    }


    if (!setupSerial(esp))
    {
        cerr
            << "ERROR: Could not configure "
            << "serial port.\n";

        close(esp);

        return 1;
    }


    // --------------------------------------------------------
    // TERMINAL SETUP
    // --------------------------------------------------------

    termios oldTerminal;
    termios newTerminal;


    tcgetattr(
        STDIN_FILENO,
        &oldTerminal
    );


    newTerminal =
        oldTerminal;


    newTerminal.c_lflag &=
        ~(
            ICANON
            | ECHO
        );


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
        oldFlags
        | O_NONBLOCK
    );


    // --------------------------------------------------------
    // CONTROLLER VARIABLES
    // --------------------------------------------------------

    Mode mode =
        MANUAL;


    bool moving =
        false;


    char currentMovement =
        'S';


    int leftSpeed =
        200;


    int rightSpeed =
        200;


    const int SPEED_STEP =
        5;


    auto lastMovementCommand =
        steady_clock::now();


    const milliseconds
        STOP_TIMEOUT(220);


    auto lastAutoUpdate =
        steady_clock::now();


    const milliseconds
        AUTO_UPDATE_INTERVAL(100);


    // --------------------------------------------------------
    // INITIAL MOTOR STATE
    // --------------------------------------------------------

    sendCommand(
        esp,
        'S'
    );


    setLeftSpeed(
        esp,
        leftSpeed
    );


    this_thread::sleep_for(
        milliseconds(60)
    );


    setRightSpeed(
        esp,
        rightSpeed
    );


    // --------------------------------------------------------
    // DISPLAY CONTROLS
    // --------------------------------------------------------

    cout << "\n";

    cout
        << "=============================\n";

    cout
        << "        ROVER CONTROL\n";

    cout
        << "=============================\n\n";


    cout
        << "Hold W = Forward\n";

    cout
        << "Hold A = Left\n";

    cout
        << "Hold D = Right\n\n";


    cout
        << "U = Left PWM +5\n";

    cout
        << "J = Left PWM -5\n";

    cout
        << "I = Right PWM +5\n";

    cout
        << "K = Right PWM -5\n";

    cout
        << "P = Show PWM\n\n";


    cout
        << "SPACE = STOP\n";

    cout
        << "M     = Manual / Auto\n";

    cout
        << "Q     = Quit\n\n";


    cout
        << "MODE: MANUAL\n";

    cout
        << "LEFT PWM:  "
        << leftSpeed
        << "\n";

    cout
        << "RIGHT PWM: "
        << rightSpeed
        << "\n";


    cout
        << "=============================\n\n";


    // --------------------------------------------------------
    // MAIN LOOP
    // --------------------------------------------------------

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
        // KEYBOARD INPUT
        // ====================================================

        if (bytes > 0)
        {
            if (
                key >= 'A'
                && key <= 'Z'
            )
            {
                key =
                    key
                    - 'A'
                    + 'a';
            }


            // ------------------------------------------------
            // EMERGENCY STOP
            // ------------------------------------------------

            if (key == ' ')
            {
                sendCommand(
                    esp,
                    'S'
                );


                moving =
                    false;


                currentMovement =
                    'S';


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


                cout
                    << "\nStopping rover...\n";


                running =
                    false;
            }


            // ------------------------------------------------
            // MODE SWITCH
            // ------------------------------------------------

            else if (key == 'm')
            {
                sendCommand(
                    esp,
                    'S'
                );


                moving =
                    false;


                currentMovement =
                    'S';


                if (mode == MANUAL)
                {
                    mode =
                        AUTO;


                    cout << "\n";

                    cout
                        << "=============================\n";

                    cout
                        << "       AUTONOMOUS MODE\n";

                    cout
                        << "=============================\n";

                    cout
                        << "Camera navigation enabled.\n";

                    cout
                        << "No autonomous reverse.\n\n";
                }

                else
                {
                    mode =
                        MANUAL;


                    // Restore manual PWM

                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );


                    this_thread::sleep_for(
                        milliseconds(20)
                    );


                    setRightSpeed(
                        esp,
                        rightSpeed
                    );


                    cout << "\n";

                    cout
                        << "=============================\n";

                    cout
                        << "          MANUAL MODE\n";

                    cout
                        << "=============================\n\n";
                }
            }


            // ------------------------------------------------
            // LEFT PWM +
            // ------------------------------------------------

            else if (key == 'u')
            {
                leftSpeed +=
                    SPEED_STEP;


                leftSpeed =
                    min(
                        255,
                        leftSpeed
                    );


                if (mode == MANUAL)
                {
                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );
                }


                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            // ------------------------------------------------
            // LEFT PWM -
            // ------------------------------------------------

            else if (key == 'j')
            {
                leftSpeed -=
                    SPEED_STEP;


                leftSpeed =
                    max(
                        0,
                        leftSpeed
                    );


                if (mode == MANUAL)
                {
                    setLeftSpeed(
                        esp,
                        leftSpeed
                    );
                }


                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            // ------------------------------------------------
            // RIGHT PWM +
            // ------------------------------------------------

            else if (key == 'i')
            {
                rightSpeed +=
                    SPEED_STEP;


                rightSpeed =
                    min(
                        255,
                        rightSpeed
                    );


                if (mode == MANUAL)
                {
                    setRightSpeed(
                        esp,
                        rightSpeed
                    );
                }


                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            // ------------------------------------------------
            // RIGHT PWM -
            // ------------------------------------------------

            else if (key == 'k')
            {
                rightSpeed -=
                    SPEED_STEP;


                rightSpeed =
                    max(
                        0,
                        rightSpeed
                    );


                if (mode == MANUAL)
                {
                    setRightSpeed(
                        esp,
                        rightSpeed
                    );
                }


                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            // ------------------------------------------------
            // SHOW PWM
            // ------------------------------------------------

            else if (key == 'p')
            {
                cout << "\n";

                cout
                    << "BASE LEFT PWM:  "
                    << leftSpeed
                    << "\n";

                cout
                    << "BASE RIGHT PWM: "
                    << rightSpeed
                    << "\n\n";
            }


            // =================================================
            // MANUAL MOVEMENT
            // =================================================

            else if (mode == MANUAL)
            {
                char command =
                    0;


                if (key == 'w')
                {
                    command =
                        'F';
                }


                // Physical steering correction:
                // keyboard LEFT sends ESP command R.

                else if (key == 'a')
                {
                    command =
                        'R';
                }


                // Physical steering correction:
                // keyboard RIGHT sends ESP command L.

                else if (key == 'd')
                {
                    command =
                        'L';
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


                        if (key == 'w')
                        {
                            cout
                                << "FORWARD"
                                << "  L="
                                << leftSpeed
                                << " R="
                                << rightSpeed
                                << "\n";
                        }


                        else if (key == 'a')
                        {
                            cout
                                << "LEFT\n";
                        }


                        else if (key == 'd')
                        {
                            cout
                                << "RIGHT\n";
                        }
                    }


                    lastMovementCommand =
                        steady_clock::now();
                }
            }
        }


        // ====================================================
        // MANUAL DEAD-MAN STOP
        // ====================================================

        if (
            mode == MANUAL
            && moving
        )
        {
            auto now =
                steady_clock::now();


            auto elapsed =
                duration_cast<milliseconds>(
                    now
                    - lastMovementCommand
                );


            if (
                elapsed
                > STOP_TIMEOUT
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


                cout
                    << "STOP\n";
            }
        }


        // ====================================================
        // AUTONOMOUS MODE
        // ====================================================

        if (mode == AUTO)
        {
            auto now =
                steady_clock::now();


            auto elapsed =
                duration_cast<milliseconds>(
                    now
                    - lastAutoUpdate
                );


            if (
                elapsed
                >= AUTO_UPDATE_INTERVAL
            )
            {
                lastAutoUpdate =
                    now;


                NavData nav;


                // --------------------------------------------
                // NO NAVIGATION FILE
                // --------------------------------------------

                if (!readNavigation(nav))
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "AUTO: NO CAMERA DATA -> STOP\n";
                }


                // --------------------------------------------
                // STALE CAMERA DATA
                // --------------------------------------------

                else if (
                    !navigationFresh(nav)
                )
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "AUTO: CAMERA DATA STALE -> STOP\n";
                }


                // --------------------------------------------
                // NAVIGATION STOP
                // --------------------------------------------

                else if (
                    nav.status == "STOP"
                    ||
                    nav.throttle <= 0.0
                )
                {
                    sendCommand(
                        esp,
                        'S'
                    );


                    cout
                        << "AUTO: STOP"
                        << " | L="
                        << nav.left
                        << "m"
                        << " C="
                        << nav.center
                        << "m"
                        << " R="
                        << nav.right
                        << "m\n";
                }


                // --------------------------------------------
                // MOVE
                // --------------------------------------------

                else
                {
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


                    this_thread::sleep_for(
                        milliseconds(10)
                    );


                    setRightSpeed(
                        esp,
                        autoRight
                    );


                    // Autonomous mode only moves
                    // the rover FORWARD.
                    //
                    // Steering is accomplished by
                    // changing track PWM.

                    sendCommand(
                        esp,
                        'F'
                    );


                    cout
                        << "AUTO: "
                        << nav.status
                        << " | L="
                        << nav.left
                        << "m"
                        << " C="
                        << nav.center
                        << "m"
                        << " R="
                        << nav.right
                        << "m"
                        << " | steer="
                        << nav.steer
                        << " | PWM="
                        << autoLeft
                        << "/"
                        << autoRight
                        << "\n";
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
        << "Controller closed.\n";


    return 0;
}
