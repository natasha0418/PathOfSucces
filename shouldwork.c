#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define GPIO pins for motors and ultrasonic sensor
#define L_TRIGGER_PIN 2         // Left Ultrasonic sensor trigger pin
#define L_ECHO_PIN 3            // Left Ultrasonic sensor echo pin
#define F_TRIGGER_PIN 8        // Front Ultrasonic sensor trigger pin
#define F_ECHO_PIN 9            // Front Ultrasonic sensor echo pin
#define LEFT_MOTOR_FORWARD 4   // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 5  // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 6  // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 7 // Right motor backward pin
#define enA 10
#define enB 11
#define rpm 70     //70
#define frpm 100   //100

// Define distance thresholds in centimeters
#define SOUND_SPEED 0.034 
const int DESIRED_DISTANCE = 45;      // cm (From the left wall)
const int FRONT_DESIRED_DISTANCE = 35; // cm (Front distance to stop)

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup function to initialize GPIO pins
void setup() {
    // Set motor pins as output
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    
    // Set ultrasonic sensor pins
    pinMode(L_TRIGGER_PIN, OUTPUT);
    pinMode(F_TRIGGER_PIN, OUTPUT);
    pinMode(L_ECHO_PIN, INPUT);
    pinMode(F_ECHO_PIN, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    analogWrite(enA, rpm);
    analogWrite(enB, rpm);

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
    analogWrite(enA, rpm);
    analogWrite(enB, rpm);
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
    delay(1000);
}

// Function to turn the robot left
void turn_left() {
    analogWrite(enA, frpm);
    analogWrite(enB, frpm);
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
    delay(1000);
}

// Function to turn the robot right
void turn_right() {
    analogWrite(enA, frpm);
    analogWrite(enB, frpm);
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
    delay(1000);
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
void wall_hugger_algorithm()
 {
  while (1) 
  {
    float leftDistance = getDistance(L_TRIGGER_PIN, L_ECHO_PIN);
    Serial.println("a");
    float frontDistance = getDistance(F_TRIGGER_PIN, F_ECHO_PIN);
    Serial.println("b");

    // Print distances for debugging
    Serial.println("Left Distance:");
    Serial.println (leftDistance);
    Serial.println("Front Distance:"); 
    Serial.println(frontDistance);

    // Wall following behavior
    if (frontDistance > FRONT_DESIRED_DISTANCE) 
    {
      move_forward();
      delay(1000);  // Keep moving forward until the front obstacle is detected
      Serial.println("c");

      if (leftDistance > DESIRED_DISTANCE) 
      {
        stop_bot();
        turn_left(); 
        move_forward();
        Serial.println("d");
      } 
      else if (leftDistance < DESIRED_DISTANCE) 
      {
        stop_bot();
        turn_right(); 
        move_forward();
        Serial.println("e");
      } 
      else 
      {
        move_forward(); // When at correct distance from the wall, continue straight
        Serial.println("f");
      }
    }

    // Front obstacle detection
    if (frontDistance <= FRONT_DESIRED_DISTANCE) 
    {
      stop_bot();
      Serial.println("g");
      while (getDistance(F_TRIGGER_PIN, F_ECHO_PIN) < 15) 
      {  // While obstacle is too close
        Serial.println("h");
        leftDistance = getDistance(L_TRIGGER_PIN, L_ECHO_PIN);  // Update left distance
        Serial.print("i");
        if (leftDistance > DESIRED_DISTANCE) 
        {
          turn_left();
          move_forward(); 
          Serial.println("j");
        } else 
        {
          turn_right();
          move_forward(); 
          Serial.println("k");
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
  lcd.clear();
  delay(10);
  wall_hugger_algorithm(); 
  delay(1000);
}
