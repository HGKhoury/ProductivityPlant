// // SENSOR TESTING

// #include "Particle.h"
// #include "Adafruit_SSD1306.h"
// #include "Adafruit_GFX.h"
// #include "splash.h" // This is our custom header containing the splash screen bitmap

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define SCREEN_ADDRESS 0x3D // OLED display address (for the 128x64)
// #define BUTTON D3 // Button pin is connected to D3
// #define PIN1 A2
// #define PIN2 A5

// SYSTEM_MODE(AUTOMATIC);
// SYSTEM_THREAD(ENABLED);

// SerialLogHandler logHandler(LOG_LEVEL_INFO);

// // Instantiate SSD1306 driver display object via I2C interface
// Adafruit_SSD1306 disp(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Servo serv1;
// Servo serv2;

// void button_press(void); // function prototype for the button press function
// void draw_button_press(void);

// bool buttonPressed = false;

// void setup() {
//   Serial.begin(9600);
//   delay(8);

//   // Initialize the button pin as an input with a pulldown resistor
//   pinMode(BUTTON, INPUT_PULLDOWN);
  
//   serv1.attach(PIN1);  // Attach the 1st servo to A2
//   serv2.attach(PIN2);   // Attach the 2nd servo to a5


//   serv1.write(0);    // Move to 0 degrees
//   delay(500);         // Wait 1 second
//   serv2.write(0);    // Move to 0 degrees
//   delay(500);         // Wait 1 second
//   serv1.write(90);   // Move to 90 degrees (center)
//   delay(500); 
//   serv2.write(90);    // Move to 0 degrees

//   // Attach an interrupt to the button pin
//   attachInterrupt(BUTTON, button_press, RISING);
//   delay(2000);

//   // Initialize the OLED display
//   if (!disp.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
//     Serial.println(F("SSD1306 allocation failed"));
//     for (;;); // Don't proceed, loop forever
//   } else {
//     Serial.println("SSD1306 allocation success");
//     draw_button_press();
//     delay(1000);
//   }
// }

// void loop() {
//   draw_button_press(); 
//   if (buttonPressed) {
//       Log.info("Button pressed!-------------------");
//       draw_button_press(); // Update the display
//       buttonPressed = false;
//   }
//   delay(1000); // Small delay to avoid CPU overload
// }

// void button_press(void) {
//   buttonPressed = true;
// }

// void draw_button_press() {
//   disp.clearDisplay();
//   disp.setTextSize(1);
//   disp.setTextColor(WHITE);
//   disp.setCursor(0, 0);
//   disp.printf("start timer!");
//   disp.display();
// }


// servo test code 

// #include "Particle.h"

// Servo myServo;

// void setup() {
//   myServo.attach(A5);  // Attach the servo to D1
// }

// void loop() {
//   myServo.write(0);    // Move to 0 degrees
//   delay(1000);         // Wait 1 second
//   myServo.write(90);   // Move to 90 degrees (center)
//   delay(1000);         // Wait 1 second
//   myServo.write(180);  // Move to 180 degrees
//   delay(1000);         // Wait 1 second
// }


/* 
 * Project Productivity Plant Display
 * Author: Hanna Khoury, Roopa Ramanujam
 * Date: October 2024
 */

#include "Particle.h"
// #line 1 "/Users/jml/Documents/programming/git_repositories/tdf-fa23-equilet/project_demonstrables/particle_workbench/potentiometer_to_oled/src/potentiometer_to_oled.ino"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "smiley.h" // contains smiley face
#include "frowny.h" // contains frowny face


SYSTEM_THREAD(ENABLED);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3D // OLED display address (for the 128x64)
#define BUTTON D3 // Button pin is connected to D3
#define PIN1 A2
#define PIN2 A5

Servo serv1;
Servo serv2;


// Instantiate SSD1306 driver display object via I2C interface; note that no reset is used
Adafruit_SSD1306 disp(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup();
void loop();
void showText(const char *text);
void productivityHandler(const char *event, const char *data);
void buttonPress(); // function prototype for the button press function
void startDeepWork();
void servoUpPosition();
void servoDownPosition();
void celebrationWiggle();
void drawHappy();
void drawFrowny();

const int angle1 = 0;
const int angle2 = 90;
const int angle3 = 180;

const int delayTime = 500;

bool buttonPressed = false;
bool isProductive = true;

unsigned long deepWorkStartTime;
const unsigned long deepWorkPeriod = 20 * 1000; // 25 minutes in milliseconds
bool deepWorkPeriodRunning = false;


unsigned long breakStartTime;
const unsigned long breakPeriod = 5 * 1000; // 25 minutes in milliseconds
bool breakRunning = false;  // Flag to indicate if the action is running

void setup(){
  pinMode(BUTTON, INPUT_PULLDOWN);
  
  serv1.attach(PIN1);  // Attach the 1st servo to A2
  serv2.attach(PIN2);   // Attach the 2nd servo to A5

  delay(500);

  servoUpPosition();

  attachInterrupt(BUTTON, buttonPress, RISING);

  Particle.subscribe("isProductive", productivityHandler);
  // Initialize the OLED display
  if (!disp.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  } 
  else {
    Serial.println("SSD1306 allocation success");
    showText("welcome to the productivity plant!");
    delay(1000);
    }
  Serial.begin(9600);
  delay(8); 
}

void loop(){
  if (buttonPressed) {
    if (!breakRunning) {
      Particle.publish("deepWorkStarted", "start");
        if (isProductive) {
            servoUpPosition();
            drawHappy();
        }
        else {
            servoDownPosition();
            drawFrowny();
        }
    }

    if (millis() - deepWorkStartTime >= deepWorkPeriod) {
        // deep work period running
        showText("break time!");
        if (!breakRunning) {
        // Start the break
            breakStartTime = millis();  // Record the start time
            breakRunning = true;  // Set the flag to indicate the action has started
        }

        unsigned long elapsedBreakTime = millis() - breakStartTime;
        if (elapsedBreakTime <= breakPeriod) {
            // break time running
            celebrationWiggle();
            Particle.publish("deepWorkStarted", "stop");
        }
        else {
            // break time over
            breakRunning = false;
            buttonPressed = false;
        }
    }
  }
  delay(100); // Pause for 1/10 second
}

void showText(const char *text) {
  disp.clearDisplay();
  disp.setTextSize(1);
  disp.setTextColor(WHITE);
  disp.setCursor(0, 10);
  disp.printf(text);
  disp.display();
}

void productivityHandler(const char *event, const char *data){
    if (strcmp(data, "no") == 0) {
      isProductive = false;
    }
    else if (strcmp(data, "yes") == 0) {
      isProductive = true;
    }
}

void buttonPress(void) {
  buttonPressed = true;
  startDeepWork();
}

void startDeepWork() {
  deepWorkStartTime = millis();
  servoUpPosition();
  showText("deep work started");
}

void servoUpPosition() {
  serv1.write(90);
  serv2.write(90);
}

void servoDownPosition() {
  serv1.write(0);
  serv2.write(180);
}

void celebrationWiggle() {
  servoUpPosition();
  delay(500);

  servoDownPosition();
  delay(500);

  servoUpPosition();
  delay(500);

  servoDownPosition();
  delay(500);

  servoUpPosition();
  delay(500);

  servoDownPosition();
  delay(500);

  servoUpPosition();
  delay(500);

  servoDownPosition();
  delay(500);
}

void drawHappy(void){
  disp.clearDisplay();
  disp.drawBitmap(0, 0, smiley, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  disp.display();
}

void drawFrowny(void){
  disp.clearDisplay();
  disp.drawBitmap(0, 0, frowny, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  disp.display();
}


// WEARABLE CODE

/* 
 * Project Productivity Plant Wearable
 * Author: Hanna Khoury, Roopa Ramanujam
 * Date: October 2024
 */

// #include <Wire.h>
// #include <MPU6050.h>

// MPU6050 mpu;

// // Variables for calibration offsets
// float ax_offset = 0;
// float ay_offset = 0;
// float az_offset = 16384;  // Z should be around 1g (16384 in raw data for ±2g range)

// // Low-pass filter variables
// float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
// float alpha = 0.8;  // Tuning parameter for low-pass filter (between 0 and 1)

// const int maxAcceptableNoiseLevel = 1000;

// bool deepWork = false;

// void calibrateSensor();
// void setup();
// void loop();
// void deepWorkHandler(const char *event, const char *data);

// void setup() {
//     Serial.begin(9600);
//     Wire.begin();

//     // Initialize loudness sensor
//     pinMode(A0, INPUT);

//     // Initialize MPU6050
//     Serial.println("Initializing MPU6050...");
//     mpu.initialize();

//     // Check connection
//     if (!mpu.testConnection()) {
//         Serial.println("MPU6050 connection failed!");
//         while (1);
//     }
//     Serial.println("MPU6050 connected!");

//     // Calibrate the sensor when stationary
//     calibrateSensor();
//     Particle.subscribe("deepWorkStarted",deepWorkHandler);
// }

// void calibrateSensor() {
//     int16_t ax, ay, az;
//     long ax_sum = 0, ay_sum = 0, az_sum = 0;
//     int n_samples = 300;  // Take 100 readings for calibration

//     // Take multiple readings when the sensor is stationary to calculate average offsets
//     for (int i = 0; i < n_samples; i++) {
//         mpu.getAcceleration(&ax, &ay, &az);
//         ax_sum += ax;
//         ay_sum += ay;
//         az_sum += az;
//         delay(100);
//     }

//     // Calculate average offsets
//     ax_offset = ax_sum / n_samples;
//     ay_offset = ay_sum / n_samples;
//     az_offset = (az_sum / n_samples) - 16384;  // Z-axis should be 1g (16384 in raw values)
    
//     Serial.println("Calibration complete.");
// }

// void loop() {
//     int16_t ax_raw, ay_raw, az_raw;
//     mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);

//     // Apply offsets (calibration)
//     float ax_g = (ax_raw - ax_offset) / 16384.0;
//     float ay_g = (ay_raw - ay_offset) / 16384.0;
//     float az_g = (az_raw - az_offset) / 16384.0;

//     // Apply low-pass filter to reduce noise
//     ax_filtered = alpha * ax_filtered + (1 - alpha) * ax_g;
//     ay_filtered = alpha * ay_filtered + (1 - alpha) * ay_g;
//     az_filtered = alpha * az_filtered + (1 - alpha) * az_g;

//     // Calculate total acceleration magnitude
//     float totalAcceleration = sqrt(ax_filtered * ax_filtered + ay_filtered * ay_filtered + az_filtered * az_filtered);

//     // Adjust threshold for stationary vs movement detection
//     float stationaryThresholdLow = 0.95;  // Slightly below 1g
//     float stationaryThresholdHigh = 1.05;  // Slightly above 1g

//     int soundLevel = analogRead(A0);
//     Serial.println(soundLevel);

//     bool isStationary = true;

//     if (totalAcceleration > stationaryThresholdHigh || totalAcceleration < stationaryThresholdLow) {
//         isStationary = false;
//         Serial.println("Movement detected");
//     } else {
//         isStationary = true;
//         Serial.println("Stationary");
//     }
//     if (deepWork){
//       if (!isStationary || soundLevel > maxAcceptableNoiseLevel) {
//           Particle.publish("isProductive", "no");
//           delay(10 * 1000);
//       }
//     } 

//     // Debugging - Print raw and filtered values
//     Serial.print("Ax: "); Serial.print(ax_filtered);
//     Serial.print(" Ay: "); Serial.print(ay_filtered);
//     Serial.print(" Az: "); Serial.print(az_filtered);
//     Serial.print(" | Total Acceleration: "); Serial.println(totalAcceleration);

//     delay(500);  // Small delay before the next reading
// }
// void deepWorkHandler(const char *event, const char *data){
//   if (strcmp(data, "start") == 0){
//     deepWork = true;
//   }
//   else if (strcmp(data, "stop") == 0){
//     deepWork = false;
//   }
// }
