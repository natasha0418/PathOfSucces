#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  // Calculate time elapsed in seconds
  lcd.clear();
  unsigned long seconds = millis() / 1000;
  lcd.setCursor(0, 0);
  lcd.print("Dir:Front");

  lcd.setCursor(10,0);
  lcd.print("0.9m/s");


  // Set cursor to the beginning of the second line
  lcd.setCursor(0, 1);  // First column, second row
  
  // Print the time elapsed in seconds
  lcd.print("Time: ");
  lcd.print(seconds);
  lcd.print("s   ");  // Extra spaces to clear any leftover digits


  
  delay(1000);  // Update every second
  
}
