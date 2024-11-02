#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdint.h>
#include <math.h>

// Define GPIO pins for motors and ultrasonic sensor
#define L_TRIGGER_PIN 2         // Left Ultrasonic sensor trigger pin
#define L_ECHO_PIN 3            // Left Ultrasonic sensor echo pin
#define F_TRIGGER_PIN 8         // Front Ultrasonic sensor trigger pin
#define F_ECHO_PIN 9            // Front Ultrasonic sensor echo pin
#define LEFT_MOTOR_FORWARD 4    // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 5   // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 6   // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 7  // Right motor backward pin

// Define distance thresholds in centimeters
#define SOUND_SPEED 0.034 
const int DESIRED_DISTANCE = 10;      // cm (From the left wall)
const int FRONT_DESIRED_DISTANCE = 5; // cm (Front distance to stop)

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup function to initialize GPIO pins and LCD
void setup() {
    // Initialize motors as output
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    
    // Set ultrasonic sensor pins
    pinMode(L_TRIGGER_PIN, OUTPUT);
    pinMode(F_TRIGGER_PIN, OUTPUT);
    pinMode(L_ECHO_PIN, INPUT);
    pinMode(F_ECHO_PIN, INPUT);
    
    Serial.begin(9600);

    // Initialize the LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

// Function to get distance from ultrasonic sensor in centimeters
float getDistance(int triggerPin, int echoPin) {
    float distance;

    // Send a 10us pulse to trigger the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Measure the pulse duration in microseconds on the echo pin
    long duration = pulseIn(echoPin, HIGH);

    // Calculate distance based on the duration
    distance = (duration * SOUND_SPEED) / 2;

    return distance;
}

// Function to drive the robot forward
void move_forward() {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    lcd.setCursor(0, 0);
    lcd.print("Dir:Front");
    lcd.setCursor(10, 0);
    lcd.print("0.9m/s");
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s ");

}

// Function to turn the robot left
void turn_left() {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    lcd.setCursor(0, 0);
    lcd.print("Dir: Left");
    lcd.setCursor(10, 0);
    lcd.print("0.9m/s");
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s ");
}

// Function to turn the robot right
void turn_right() {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    lcd.setCursor(0, 0);
    lcd.print("Dir:Right");
    lcd.setCursor(10, 0);
    lcd.print("0.9m/s");
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s ");
}

// Function to stop the robot
void stop_bot() {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    lcd.setCursor(0, 0);
    lcd.print("Dir: Halt");
    lcd.setCursor(10, 0);
    lcd.print("0.0m/s");
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s ");
    delay(1000);
}

// Wall Hugger Algorithm
void wall_hugger_algorithm() {
  while (1) {
    float leftDistance = getDistance(L_TRIGGER_PIN, L_ECHO_PIN);
    float frontDistance = getDistance(F_TRIGGER_PIN, F_ECHO_PIN);

    // Print distances for debugging
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.print(" Front Distance: "); 
    Serial.println(frontDistance);

    // Wall following behavior
    if (frontDistance > FRONT_DESIRED_DISTANCE) {
      move_forward();  // Move forward until front obstacle is detected

      if (leftDistance > DESIRED_DISTANCE) {
        stop_bot();
        turn_left(); 
        move_forward();
      } 
      else if (leftDistance < DESIRED_DISTANCE) {
        stop_bot();
        turn_right(); 
        move_forward();
      } 
      else {
        move_forward(); // Continue straight at correct distance
      }
    }

    // Front obstacle detection
    if (frontDistance <= FRONT_DESIRED_DISTANCE) {
      stop_bot();
      while (getDistance(F_TRIGGER_PIN, F_ECHO_PIN) < 15) {  // While obstacle is too close
        leftDistance = getDistance(L_TRIGGER_PIN, L_ECHO_PIN);  // Update left distance
        if (leftDistance > DESIRED_DISTANCE) {
          turn_left();  
        } else {
          turn_right();  
        }
      }
    }

    // Update elapsed time on the LCD
    lcd.setCursor(10, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s ");

    delay(500);  // Small delay for sensor readings update
  }
}

// Main function to control robot's movement based on distance
void loop() {
  wall_hugger_algorithm(); 
}
