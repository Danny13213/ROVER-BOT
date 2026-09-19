#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <string>

using namespace std;
using namespace std::chrono;


// ==========================================
// MODE
// ==========================================

enum Mode {
    MANUAL,
    AUTO
};


// ==========================================
// SEND MOVEMENT COMMAND
// ==========================================

void sendCommand(int esp, char command)
{
    write(esp, &command, 1);
}


// ==========================================
// SET LEFT MOTOR SPEED
//
// ESP32:
// A200 = left PWM 200
// ==========================================

void setLeftSpeed(int esp, int speed)
{
    if (speed < 0)
        speed = 0;

    if (speed > 255)
        speed = 255;

    string command =
        "A" + to_string(speed) + "\n";

    write(
        esp,
        command.c_str(),
        command.length()
    );
}


// ==========================================
// SET RIGHT MOTOR SPEED
//
// ESP32:
// C200 = right PWM 200
// ==========================================

void setRightSpeed(int esp, int speed)
{
    if (speed < 0)
        speed = 0;

    if (speed > 255)
        speed = 255;

    string command =
        "C" + to_string(speed) + "\n";

    write(
        esp,
        command.c_str(),
        command.length()
    );
}


// ==========================================
// SERIAL SETUP
// ==========================================

bool setupSerial(int fd)
{
    termios tty{};

    if (tcgetattr(fd, &tty) != 0)
        return false;

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    return tcsetattr(
        fd,
        TCSANOW,
        &tty
    ) == 0;
}


// ==========================================
// MAIN
// ==========================================

int main()
{
    // ======================================
    // OPEN ESP32
    // ======================================

    const char* espDevice =
        "/dev/ttyUSB0";

    int esp = open(
        espDevice,
        O_RDWR | O_NOCTTY
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
            << "ERROR: Could not configure serial port.\n";

        close(esp);

        return 1;
    }


    // ======================================
    // CONFIGURE SSH TERMINAL
    // ======================================

    termios oldTerminal;
    termios newTerminal;

    tcgetattr(
        STDIN_FILENO,
        &oldTerminal
    );

    newTerminal = oldTerminal;

    // Disable normal line input and echo
    newTerminal.c_lflag &=
        ~(ICANON | ECHO);

    newTerminal.c_cc[VMIN] = 0;
    newTerminal.c_cc[VTIME] = 0;

    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &newTerminal
    );


    // Make keyboard input non-blocking

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


    // ======================================
    // VARIABLES
    // ======================================

    Mode mode = MANUAL;

    bool moving = false;

    char currentMovement = 'S';


    // ======================================
    // MOTOR SPEEDS
    // ======================================

    int leftSpeed = 200;
    int rightSpeed = 200;

    const int SPEED_STEP = 5;


    // ======================================
    // DEAD-MAN TIMER
    // ======================================

    auto lastMovementCommand =
        steady_clock::now();

    const milliseconds
        STOP_TIMEOUT(220);


    // ======================================
    // INITIALIZE ROVER
    // ======================================

    // Start stopped
    sendCommand(esp, 'S');

    // Send starting PWM values
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


    // ======================================
    // DISPLAY CONTROLS
    // ======================================

    cout << "\n";

    cout
        << "=============================\n";

    cout
        << "        ROVER CONTROL\n";

    cout
        << "=============================\n";

    cout << "\n";

    cout
        << "Hold W = Forward\n";

    cout
        << "Hold A = Left\n";

    cout
        << "Hold D = Right\n";

    cout << "\n";

    cout
        << "U = Left PWM +5\n";

    cout
        << "J = Left PWM -5\n";

    cout
        << "I = Right PWM +5\n";

    cout
        << "K = Right PWM -5\n";

    cout
        << "P = Show PWM\n";

    cout << "\n";

    cout
        << "SPACE = STOP\n";

    cout
        << "M     = Manual / Auto\n";

    cout
        << "Q     = Quit\n";

    cout << "\n";

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


    // ======================================
    // MAIN LOOP
    // ======================================

    bool running = true;

    while (running)
    {
        char key;

        ssize_t bytes =
            read(
                STDIN_FILENO,
                &key,
                1
            );


        // ==================================
        // KEY RECEIVED
        // ==================================

        if (bytes > 0)
        {
            // Convert uppercase to lowercase

            if (
                key >= 'A' &&
                key <= 'Z'
            )
            {
                key =
                    key - 'A' + 'a';
            }


            // ==============================
            // EMERGENCY STOP
            // ==============================

            if (key == ' ')
            {
                sendCommand(
                    esp,
                    'S'
                );

                moving = false;

                currentMovement = 'S';

                cout
                    << "\n*** STOP ***\n";
            }


            // ==============================
            // QUIT
            // ==============================

            else if (key == 'q')
            {
                sendCommand(
                    esp,
                    'S'
                );

                cout
                    << "\nStopping rover...\n";

                running = false;
            }


            // ==============================
            // MODE TOGGLE
            // ==============================

            else if (key == 'm')
            {
                // Stop before switching modes

                sendCommand(
                    esp,
                    'S'
                );

                moving = false;

                currentMovement = 'S';


                if (mode == MANUAL)
                {
                    mode = AUTO;

                    cout << "\n";

                    cout
                        << "=============================\n";

                    cout
                        << "       AUTONOMOUS MODE\n";

                    cout
                        << "=============================\n";

                    cout
                        << "Rover stopped.\n";

                    cout
                        << "Autonomous navigation "
                        << "not enabled yet.\n\n";
                }

                else
                {
                    mode = MANUAL;

                    cout << "\n";

                    cout
                        << "=============================\n";

                    cout
                        << "          MANUAL MODE\n";

                    cout
                        << "=============================\n\n";
                }
            }


            // ==============================
            // LEFT PWM +
            // ==============================

            else if (key == 'u')
            {
                leftSpeed += SPEED_STEP;

                if (leftSpeed > 255)
                    leftSpeed = 255;

                setLeftSpeed(
                    esp,
                    leftSpeed
                );

                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            // ==============================
            // LEFT PWM -
            // ==============================

            else if (key == 'j')
            {
                leftSpeed -= SPEED_STEP;

                if (leftSpeed < 0)
                    leftSpeed = 0;

                setLeftSpeed(
                    esp,
                    leftSpeed
                );

                cout
                    << "LEFT PWM: "
                    << leftSpeed
                    << "\n";
            }


            // ==============================
            // RIGHT PWM +
            // ==============================

            else if (key == 'i')
            {
                rightSpeed += SPEED_STEP;

                if (rightSpeed > 255)
                    rightSpeed = 255;

                setRightSpeed(
                    esp,
                    rightSpeed
                );

                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            // ==============================
            // RIGHT PWM -
            // ==============================

            else if (key == 'k')
            {
                rightSpeed -= SPEED_STEP;

                if (rightSpeed < 0)
                    rightSpeed = 0;

                setRightSpeed(
                    esp,
                    rightSpeed
                );

                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n";
            }


            // ==============================
            // DISPLAY PWM
            // ==============================

            else if (key == 'p')
            {
                cout << "\n";

                cout
                    << "LEFT PWM:  "
                    << leftSpeed
                    << "\n";

                cout
                    << "RIGHT PWM: "
                    << rightSpeed
                    << "\n\n";
            }


            // ==============================
            // MANUAL MOVEMENT
            // ==============================

            else if (mode == MANUAL)
            {
                char command = 0;


                // FORWARD
                if (key == 'w')
                {
                    command = 'F';
                }


                // IMPORTANT:
                //
                // Your physical left/right
                // directions are reversed.
                //
                // A should physically turn LEFT,
                // so send R to the ESP32.
                else if (key == 'a')
                {
                    command = 'R';
                }


                // D should physically turn RIGHT,
                // so send L to the ESP32.
                else if (key == 'd')
                {
                    command = 'L';
                }


                if (command != 0)
                {
                    // Send only when the
                    // direction changes

                    if (
                        !moving ||
                        command != currentMovement
                    )
                    {
                        sendCommand(
                            esp,
                            command
                        );

                        currentMovement =
                            command;

                        moving = true;


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
                                << "LEFT"
                                << "  L="
                                << leftSpeed
                                << " R="
                                << rightSpeed
                                << "\n";
                        }

                        else if (key == 'd')
                        {
                            cout
                                << "RIGHT"
                                << "  L="
                                << leftSpeed
                                << " R="
                                << rightSpeed
                                << "\n";
                        }
                    }


                    // Refresh dead-man timer

                    lastMovementCommand =
                        steady_clock::now();
                }
            }
        }


        // ==================================
        // DEAD-MAN STOP
        // ==================================

        if (
            mode == MANUAL &&
            moving
        )
        {
            auto now =
                steady_clock::now();

            auto elapsed =
                duration_cast<milliseconds>(
                    now -
                    lastMovementCommand
                );


            if (
                elapsed >
                STOP_TIMEOUT
            )
            {
                sendCommand(
                    esp,
                    'S'
                );

                moving = false;

                currentMovement = 'S';

                cout
                    << "STOP\n";
            }
        }


        // ==================================
        // AUTONOMOUS MODE
        // ==================================

        if (mode == AUTO)
        {
            /*
             * Autonomous navigation will
             * go here.
             *
             * Eventually:
             *
             * Astra depth camera
             *       |
             *       v
             *
             * LEFT / CENTER / RIGHT
             * distance measurements
             *
             *       |
             *       v
             *
             * Decide:
             *
             * F = forward
             * R = physical left
             * L = physical right
             * S = stop
             *
             *       |
             *       v
             *
             * ESP32
             */
        }


        // Prevent CPU from running at 100%

        this_thread::sleep_for(
            milliseconds(5)
        );
    }


    // ======================================
    // CLEANUP
    // ======================================

    sendCommand(
        esp,
        'S'
    );


    // Restore terminal

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


    cout
        << "Controller closed.\n";


    return 0;
}
