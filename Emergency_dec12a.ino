// ========== Michael J. Miller ===========
//    Final Project Component, Option#1
// ****************************************
// ****************************************
//   Due to difficulties with Cayenne,
//   dashboard not refreshing and
//   site outages, the project was developed
//   using the Arduino IoT Cloud.
//   https://cloud.arduino.cc
// ****************************************
// ****************************************

#include "arduino_secrets.h"
#include "thingProperties.h"
#include <LiquidCrystal_I2C.h>

// Initialize lcd display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Current state variable
String current_state = "normal"; // Set initial state to normal

// Crosswalk counter start int
int walk_counter = 10;

// Last time millis was updated
unsigned long previousMillis = 0;

// Normal() interval and state of lights
int normal_state = 0;
int normal_interval = 2000;

// Display messages
String class_name    = "=== CEIS114 ====";
String dont_walk_msg = "== Do Not Walk =";
String emergency_msg = ">>  Emergency <<";
String walk_msg      = "===== Walk =====";
String bsp           = "                ";
String count_msg     = "Count = ";

//====================== Pin setup =================================
const int red_LED1      = 14; // The red LED1 is wired to ESP32 board pin GPIO14
const int yellow_LED1   = 12; // The yellow LED1 is wired to ESP32 board pin GPIO12
const int green_LED1    = 13; // The green LED1 is wired to ESP32 board pin GPIO13
const int red_LED2      = 25; // The red LED2 is wired to Mega board pin GPIO25
const int yellow_LED2   = 26; // The yellow LED2 is wired to Mega board pin GPIO 26
const int green_LED2    = 27; // The green LED2 is wired to Mega board pin GPIO 27
const int corsswalk_btn = 19; // Cross Walk button
const int blue_LED      = 16; // GPIO16  to trigger the emergency button
const int buzzer        = 32; // GPIO32 to connect the Buzzer
//====================== End pin setup =============================

void setup() {
  Serial.begin(9600);
  delay(1500); // This delay gives the chance for the Serial Monitor to start

  // Arduino IoT Cloud initialize
  initProperties(); // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  //======================= LCD Initialization ==================================
  lcd.init();           // Initialize the lcd
  lcd.backlight();      // Turn on LCD backlight
  lcd.setCursor(0, 0);  // Set cursor at column#1 and Row #1
  lcd.print(class_name);

  //========================== Pinmode setup ====================================
  pinMode(buzzer, OUTPUT);
  pinMode(red_LED1, OUTPUT);     // initialize digital pin 14 (Red LED1) as an output.
  pinMode(yellow_LED1, OUTPUT);  // initialize digital pin 12 (yellow LED1) as an output.
  pinMode(green_LED1, OUTPUT);   // initialize digital pin 13 (green LED1) as an output.
  pinMode(red_LED2, OUTPUT);     // initialize digital pin 25(Red LED2) as an output.
  pinMode(yellow_LED2, OUTPUT);  // initialize digital pin 26 (yellow LED2) as an output.
  pinMode(green_LED2, OUTPUT);   // initialize digital pin 27 (green LED2) as an output.
  pinMode(corsswalk_btn, INPUT_PULLUP);  // 0=pressed, 1 = unpressed button
  pinMode(blue_LED, OUTPUT);

  digitalWrite(blue_LED, LOW);
}

void loop() {
  // Checks Arduino IoT Cloud dashboard for changes
  ArduinoCloud.update();

  // Update status light on dashboard
  if (current_state == "emergency") {
    status = true;
  } else status = false;  

  if (current_state == "emergency") {
    Emergency(); // Flash red lights and signal Emergency
  } else if (current_state == "walk") {
    Walk();      // Flash Red lights and signal Walk
  } else if (current_state == "normal") {
    Normal();    // Normal traffic light operation
  } else current_state = "normal";
}

// Called when SWITCH_ON var is changed on dashboard
void onSWITCHONChange()  {
  if (SWITCH_ON) {
    current_state = "emergency";
    digitalWrite(blue_LED, HIGH); // Turn on BLUE LED
    reset_lights();
  } else {
    current_state = "normal";
    digitalWrite(blue_LED, LOW); // Turn off BLUE LED
    reset_lights();
  }
}

// Reset all traffic lights to off
void reset_lights() {
  digitalWrite(red_LED1, LOW);    // This should turn on the RED LED1
  digitalWrite(yellow_LED1, LOW);  // This should turn off the YELLOW LED1
  digitalWrite(green_LED1, LOW);   // This should turn off the GREEN LED1
  digitalWrite(red_LED2, LOW);     // This should turn off the RED LED2
  digitalWrite(yellow_LED2, LOW);  // This should turn off the YELLOW LED2
  digitalWrite(green_LED2, LOW);  // This should turn on the GREEN LED2
  digitalWrite(buzzer, LOW);
  normal_state = 0;
}

// Toggle Red lights and buzzer state
void toggle_buzzer_lights(String message) {
  // get current state of lights
  int state = digitalRead(red_LED1);

  // toggle lights and buzzer
  digitalWrite(red_LED1, !state);
  digitalWrite(red_LED2, !state);
  digitalWrite(buzzer, !state);

  // Flash "Dont Walk" message on lcd
  if (state) {
    lcd.setCursor(0, 1);
    lcd.print(message);
  } else {
    lcd.setCursor(0, 1);
    lcd.println(bsp);
  }
}

//=========== Normal traffic light operation ======================
void Normal() {
  // Current milliseconds used for timing delay
  unsigned long currentMillis = millis();
  
  // read the cross walk button value
  // cross walk will only activate in normal state
  if (!digitalRead(corsswalk_btn)) current_state = "walk";
  
  if (currentMillis - previousMillis > normal_interval) {
    if (normal_state == 0) {
      //=========== Turn on the red_LED1 and green_LED2 ===============================
      digitalWrite(red_LED1, HIGH);    // This should turn on the RED LED1
      digitalWrite(red_LED2, LOW);     // This should turn off the RED LED2
      digitalWrite(green_LED2, HIGH);  // This should turn on the GREEN LED2
      normal_state += 1;
      normal_interval = 2000;
    } else if (normal_state == 1) {
      // //============ Turn on the red_LED1 and yellow_LED2 ==============================
      digitalWrite(yellow_LED2, HIGH);  // This should turn on the YELLOW LED2
      digitalWrite(green_LED2, LOW);    // This should turn off the GREEN LED2
      normal_state += 1;
    } else if (normal_state == 2 || normal_state == 5) {
      //============= Both are RED for 1 second =========================================
      digitalWrite(yellow_LED1, LOW);  // This should turn off the YELLOW LED1
      digitalWrite(red_LED2, HIGH);    // This should turn on the RED LED2
      digitalWrite(yellow_LED2, LOW);  // This should turn off the YELLOW LED2
      if (normal_state == 2) {
        normal_state += 1;
      } else if (normal_state == 5) {
        normal_state = 0;
      }
      normal_interval = 1000;
    } else if (normal_state == 3) {
      //================= Turn on green_LED1 and red_LED2 ==============================
      digitalWrite(red_LED1, LOW);     // This should turn off the RED LED1
      digitalWrite(green_LED1, HIGH);  // This should turn on the GREEN LED1
      normal_state += 1;
      normal_interval = 2000;
    } else if (normal_state == 4) {
      //==================== Turn on yellow_LED1 and red_LED2 =========================
      digitalWrite(yellow_LED1, HIGH);  //  This should turn on the YELLOW LED1
      digitalWrite(green_LED1, LOW);    //  This should turn off the GREEN LED1
      normal_state += 1;
    }           

    lcd.setCursor(0, 0);  // Set cursor at column#1 and Row #1
    lcd.print(class_name);
    lcd.setCursor(0, 1);       // set the cursor to column 1, line 2
    lcd.print(dont_walk_msg);       // Display = Do Not Walk = on the LCD.
    Serial.println(dont_walk_msg);  // Also, Display = Do Not Walk = on the Serial Monitor
    
    previousMillis = currentMillis;
  }
  if (current_state != "normal") {
    Serial.println("reset from normal");
    reset_lights();
    normal_state = 0;
  }
}

void Walk() {
  const int interval = 1000;
  unsigned long currentMillis = millis();

  //=================== flashing light and sounding buzzer =============
  if (currentMillis - previousMillis > interval) {
    //=================== flashing light and sounding buzzer =============
    toggle_buzzer_lights(walk_msg);

    lcd.setCursor(0, 0);
    lcd.println(count_msg + String(walk_counter) + bsp);  // Print the counter value

    Serial.println(walk_msg);
    Serial.println(count_msg + String(walk_counter));

    // Decrement crosswalk counter
    walk_counter--;
    // reset counter and current state if countdown finished
    if (walk_counter < 0) {
      walk_counter = 10;
      current_state = "normal";
    }
    previousMillis = currentMillis;
  }
  if (current_state != "walk") reset_lights();
}

void Emergency() {
  const int interval = 500;
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) {
    //=================== flashing light and sounding buzzer =============
    toggle_buzzer_lights(dont_walk_msg);

    // Send Emergency to Serial monitor
    Serial.println(emergency_msg);
    Serial.println(dont_walk_msg);
    lcd.setCursor(0, 0);
    lcd.print(emergency_msg);
    
    previousMillis = currentMillis;
  }
  if (current_state != "emergency") reset_lights();
}
