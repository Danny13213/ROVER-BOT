// Commands from Jetson:
//
// F = forward
// B = backward
// L = left
// R = right
// S = stop
//
// A### = set LEFT motor speed
// C### = set RIGHT motor speed
//
// Examples:
// A120 = left motor PWM 120
// C105 = right motor PWM 105


#define PWMA 27
#define AIN1 25
#define AIN2 26

#define PWMB 15
#define BIN1 32
#define BIN2 33

#define STBY 12


// Independent motor speeds
int leftSpeed  = 120;
int rightSpeed = 120;


void setup() {

  Serial.begin(115200);

  // Makes Serial.parseInt() return quickly
  Serial.setTimeout(50);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  stopMotors();

  Serial.println("Rover ready");
}


void loop() {

  if (Serial.available()) {

    char command = Serial.read();

    switch (command) {

      // =========================
      // MOVEMENT
      // =========================

      case 'F':
        forward();
        break;

      case 'B':
        backward();
        break;

      case 'L':
        turnLeft();
        break;

      case 'R':
        turnRight();
        break;

      case 'S':
        stopMotors();
        break;


      // =========================
      // LEFT MOTOR SPEED
      // =========================

      case 'A':
      {
        int newSpeed = Serial.parseInt();

        if (newSpeed >= 0 && newSpeed <= 255) {

          leftSpeed = newSpeed;

          Serial.print("Left PWM = ");
          Serial.println(leftSpeed);
        }

        break;
      }


      // =========================
      // RIGHT MOTOR SPEED
      // =========================

      case 'C':
      {
        int newSpeed = Serial.parseInt();

        if (newSpeed >= 0 && newSpeed <= 255) {

          rightSpeed = newSpeed;

          Serial.print("Right PWM = ");
          Serial.println(rightSpeed);
        }

        break;
      }
    }
  }
}


// =============================
// FORWARD
// =============================

void forward() {

  // Left track forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Right track forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}


// =============================
// BACKWARD
// =============================

void backward() {

  // Left track backward
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // Right track backward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}


// =============================
// TURN LEFT
// =============================

void turnLeft() {

  // Left track backward
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // Right track forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, leftSpeed / 2);
  analogWrite(PWMB, rightSpeed);
}


// =============================
// TURN RIGHT
// =============================

void turnRight() {

  // Left track forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Right track backward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed / 2);
}


// =============================
// STOP
// =============================

void stopMotors() {

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
