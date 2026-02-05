#include <Servo.h>

/*
Initialise:
Set servo to initial angle
Set sensor to low

Start:
Move forward until reached set sensor distance
If stopped for >= STOP_TIME_TO_LAUNCH, trigger the servo
- Servo should be connected to a launching mechanism
Set flag launched to true

End:
After launch, floor reverse gear

Untested:
MotorPin1: Low; MotorPin2: Low => Stop
MotorPin1: High; MotorPin2: High => Go
MotorPin1: Low; MotorPin2: High => Rev
MotorPin1: High; MotorPin2: Low => ?idk
*/

// Constants
int TRIG_PIN = 13;
int ECHO_PIN = 12;
int MOTOR_PIN1 = 6;
int MOTOR_PIN2 = 5;
float SPEED_OF_SOUND = 0.0345;
int STOPPING_DISTANCE = 10; 
int STOP_TIME_TO_LAUNCH = 2000;

// Servo Info
Servo sv;
int SERVO_PIN = 9;
int INITIAL_ANGLE = 0;
int LAUNCH_ANGLE = 180;

// Others
bool launched = false;
bool stopped = false;
float since_stopped = 0;

void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  sv.attach(SERVO_PIN);
  sv.write(INITIAL_ANGLE);
  Serial.begin(9600);

}

void loop() {
  // Should reverse. Untested
  if (launched) {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
    return;
  }

  // Sends ultrasonic output 
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the return signal
  int microsecs = pulseIn(ECHO_PIN, HIGH);
  float cms = microsecs * SPEED_OF_SOUND / 2;

  //Serial.print("Distance of: "); Serial.println(cms);
  if (cms <= STOPPING_DISTANCE) {
    // Stop the motor
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);

    if (!stopped) {
      since_stopped = millis();
      stopped = true;
    }
    //Serial.print(millis() - since_stopped); Serial.println("s");

    if (millis() - since_stopped >= STOP_TIME_TO_LAUNCH) {
      delay(500);
      sv.write(LAUNCH_ANGLE);
      delay(500);
      sv.write(INITIAL_ANGLE);
      launched = true;
    }

  } else {
    stopped = false;
    since_stopped = 0;
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, HIGH);
  }

  // Delay required for ultra sonic sensor to read
  delay(10);
}
