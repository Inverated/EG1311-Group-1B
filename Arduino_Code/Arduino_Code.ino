#include <Servo.h>

/*
Initialise:
Set servo to initial angle
Set sensor and motor to low

Start:
Move forward until reached set sensor distance
If stopped for >= STOP_TIME_TO_LAUNCH, trigger the servo
- Servo should be connected to a launching mechanism
Set launched flag launched to true

End:
After launch, floor reverse gear

Motor Config:
MotorPin1: Low;   MotorPin2: Low OR EnablePin: Low  => Stop
MotorPin1: High;  MotorPin2: Low                    => Go
MotorPin1: Low;   MotorPin2: High                   => Rev
MotorPin1: High;  MotorPin2: High                   => Not go (+ve & -ve same same level, no potential diff)
*/

// Ultrasonic Sensor
int TRIG_PIN = 12;
int ECHO_PIN = 13;
float SPEED_OF_SOUND = 0.0345;
int STOPPING_DISTANCE = 7; 
int STOP_TIME_TO_LAUNCH = 1000;

/*
//H-Bridge Motor Driver
0-255. Using constant speed for both motor to simplify things
Any value below 160 causes motor to beep
*/
int MOTOR_SPEED = 200;  

// Left Motor
int LEFT_DRIVER_PIN1 = 4;
int LEFT_DRIVER_PIN2 = 2;
int LEFT_ENABLE_PIN = 5;    // PWN for controlling speed

// Right Motor
int RIGHT_DRIVER_PIN1 = 8;
int RIGHT_DRIVER_PIN2 = 7;
int RIGHT_ENABLE_PIN = 6;   // PWN for controlling speed

// Servo Motor
Servo sv;
int SERVO_PIN = 3;          // PWM for controlling angle
int INITIAL_ANGLE = 230;
int LAUNCH_ANGLE = 90;

// Others
bool launched = false;
bool stopped = false;
float since_stopped = 0;

void setup() {
  delay(2000);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);  // Set receive to low to not trigger on start

  pinMode(LEFT_DRIVER_PIN1, OUTPUT);
  pinMode(LEFT_DRIVER_PIN2, OUTPUT);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);

  pinMode(RIGHT_DRIVER_PIN1, OUTPUT);
  pinMode(RIGHT_DRIVER_PIN2, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);

  analogWrite(LEFT_ENABLE_PIN, MOTOR_SPEED);
  digitalWrite(LEFT_DRIVER_PIN1, LOW);
  digitalWrite(LEFT_DRIVER_PIN2, LOW);

  analogWrite(RIGHT_ENABLE_PIN, MOTOR_SPEED);
  digitalWrite(RIGHT_DRIVER_PIN1, LOW);
  digitalWrite(RIGHT_DRIVER_PIN2, LOW);

  sv.attach(SERVO_PIN);
  sv.write(INITIAL_ANGLE);

  Serial.begin(9600);
}

void loop() {
  if (launched) {
    // Reverse
    analogWrite(LEFT_ENABLE_PIN, MOTOR_SPEED);
    digitalWrite(LEFT_DRIVER_PIN1, LOW);
    digitalWrite(LEFT_DRIVER_PIN2, HIGH);

    analogWrite(RIGHT_ENABLE_PIN, MOTOR_SPEED);
    digitalWrite(RIGHT_DRIVER_PIN1, LOW);
    digitalWrite(RIGHT_DRIVER_PIN2, HIGH);
    return;
  }

  // Sends ultrasonic output 
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the return signal
  int microsecs = pulseIn(ECHO_PIN, HIGH);
  float cms = microsecs * SPEED_OF_SOUND / 2;

  Serial.print("Distance of: "); Serial.println(cms);
  if (cms <= STOPPING_DISTANCE) {
    // Stop the motor
    analogWrite(LEFT_ENABLE_PIN, 0);
    analogWrite(RIGHT_ENABLE_PIN, 0);

    if (!stopped) {
      since_stopped = millis();
      stopped = true;
    }

    // Ensure it is not a false stop before launching
    if (millis() - since_stopped >= STOP_TIME_TO_LAUNCH) {
      delay(500);
      sv.write(LAUNCH_ANGLE);
      delay(500);
      sv.write(INITIAL_ANGLE);
      launched = true;
      delay(300);
    }

  } else {
    // Reset flags
    stopped = false;
    since_stopped = 0;

    // Go
    analogWrite(LEFT_ENABLE_PIN, MOTOR_SPEED);
    digitalWrite(LEFT_DRIVER_PIN1, HIGH);
    digitalWrite(LEFT_DRIVER_PIN2, LOW);
    
    analogWrite(RIGHT_ENABLE_PIN, MOTOR_SPEED);
    digitalWrite(RIGHT_DRIVER_PIN1, HIGH);
    digitalWrite(RIGHT_DRIVER_PIN2, LOW);
  }

  // Delay required for ultrasonic sensor to read
  delay(10);
}
