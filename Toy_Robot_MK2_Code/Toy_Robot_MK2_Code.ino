#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

#include <SparkFun_TB6612.h>
#include <esp_timer.h>

// Pin definitions
#define PWMA 32 //pin 7 
#define AIN2 33 //pin 8
#define AIN1 25 //pin 9
#define STBY1 3 //pin 10
#define BIN1 19 //pin 31
#define BIN2 18 //pin 30
#define PWMB 27 //pin 28 
#define PWMC 4 //pin 26
#define CIN1 13 //pin 37
#define CIN2 15 //pin 33
#define STBY2 12 //pin 15

// Motor offsets (1 or -1 to adjust wiring/polarity)
const int offsetA = 1;
const int offsetB = 1;
const int offsetC = 1;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY1); // right motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY1); // left motor
Motor motor3 = Motor(CIN1, CIN2, PWMC, offsetC, STBY2);

// Encoder pins
const int enc_pin1 = 34; 
const int enc_pin2 = 35; 

// PID controller globals
float eIntegral = 0;
float ePrevious = 0;
long prevT;
const int motor_power = 200;
unsigned long ticksA = 0;
unsigned long ticksB = 0;

// --- New State Machine Variables --- //
enum Command { NONE, FORWARD, BACKWARD, LEFT_CMD, RIGHT_CMD, TRIANGLE_CMD, CROSS_CMD };
Command currentCommand = NONE;
unsigned long commandStartTime = 0;
const unsigned long debounceDelay = 20; // milliseconds

#include <U8g2lib.h>
#include <stdlib.h> // Required for random() and randomSeed()
#include <Math.h>
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <U8g2lib.h>


#define DEV_I2C Wire
#define SerialPort Serial

// XSHUT pin is tied HIGH, so we're not controlling it via a digital pin
const int XSHUT_NOT_USED = -1; // Or any other value that signifies it's not used
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, XSHUT_NOT_USED);

// Define a class to represent an eye
class Eye {
public:
    int width;  // Width of the eye
    int height; // Height of the eye
    int x;      // Horizontal position of the eye's center
    int y;      // Vertical position of the eye's center
    int initial_x; // Store initial x position
    int initial_y; // Store initial y position
    int initial_height; // Store initial height

    // Constructor to initialize an eye with given dimensions and position
    Eye(int init_width, int init_height, int init_x, int init_y)
        : width(init_width), height(init_height), x(init_x), y(init_y), initial_x(init_x), initial_y(init_y), initial_height(init_height) {}

    // Method to draw the eye on the OLED screen
    void draw(U8G2 &u8g2) {
        int x_pos = x - width / 2; // Calculate the top-left x coordinate of the eye
        int y_pos = y - height / 2; // Calculate the top-left y coordinate of the eye
        u8g2.drawBox(x_pos, y_pos, width, height); // Draw a box representing the eye
    }

    // Method to move the eye by a given amount
    void move(int dx, int dy) {
        x += dx; // Adjust the horizontal position
        y += dy; // Adjust the vertical position
    }

    // Method to move the eye to its initial position
    void moveToInitialPosition() {
        x = initial_x;
        y = initial_y;
    }

    // Method to simulate closing the eye
    void close() {
        height = 1; // Set height to a minimal value to appear closed
    }

    // Method to simulate opening the eye
    void open() {
        height = initial_height; // Restore the initial height
    }

    // Method to gradually close the eye
    void graduallyClose(int step) {
        if (height > 1) {
            height -= step;
            if (height < 1) {
                height = 1;
            }
        }
    }

    // Method to gradually open the eye
    void graduallyOpen(int step) {
        if (height < initial_height) {
            height += step;
            if (height > initial_height) {
                height = initial_height;
            }
        }
    }
};

// Define states for the eye movement sequence
enum class EyeState {
    CENTER_WAIT,
    MOVE_TO_TARGET,
    PAUSE_MOVEMENT,
    HOLD_TARGET,
    RETURN_TO_CENTER,
    CLOSE_EYES,
    OPEN_EYES,
    SLEEPING,
    WAKING_UP
};

// Initialize the OLED display
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0);

// Reference values for eye dimensions and screen
const int ref_eye_width = 40;            // Default width of an eye
const int ref_eye_height = 40;           // Default height of an eye - ADJUSTED
const int ref_space_between_eyes = 10;   // Space between the eyes
const int screen_width = 128;            // Width of the OLED screen
const int screen_height = 64;            // Height of the OLED screen

// Initial center positions
const int initial_left_eye_x = screen_width / 4; // Center left
const int initial_right_eye_x = screen_width * 3 / 4; // Center right
const int initial_eye_y = screen_height / 2; // Center vertical

// Create two Eye objects for the left and right eyes
Eye left_eye(ref_eye_width, ref_eye_height, initial_left_eye_x, initial_eye_y);
Eye right_eye(ref_eye_width, ref_eye_height, initial_right_eye_x, initial_eye_y);

// Variables for timing and movement
unsigned long startMoveMillis = 0;    // Records the time when movement/hold begins
unsigned long holdStartTime = 0;
int movementStep = 1;             // Step size for each movement of the eyes

EyeState currentEyeState = EyeState::CENTER_WAIT;
EyeState previousEyeState = EyeState::CENTER_WAIT;
unsigned long stateStartTime = 0; // To keep track of time within each state
unsigned long lastMovementSequenceTime = 0; // Track the last time the movement sequence started
const unsigned long BLINK_DURATION = 200; // milliseconds - duration of the closed state
const unsigned long BLINK_TRANSITION_DURATION = 100; // milliseconds - duration of closing/opening
const unsigned long BLINK_INTERVAL = 3500; // Blink every 3 seconds
unsigned long lastBlinkTime = 0;
unsigned long pauseMovementDuration = 0; // Duration of the pause during movement
unsigned long pauseMovementStartTime = 0; // Time when the pause during movement started

// Adjusted target boundaries to ensure more significant movement
int x_max = screen_width - left_eye.width - ref_space_between_eyes - right_eye.width;
int x_min = left_eye.width / 2;
int y_max = screen_height - left_eye.height / 2;
int y_min = left_eye.height / 2;
int target_x;
int target_y;

unsigned long lastButtonPress = 0;
//unsigned long IDLE_TIME = 600000; //Idle time is set to 10 minutes
unsigned long IDLE_TIME_LIMIT = 30000; 

void setup()
{
  SerialPort.begin(9600); // Initialize serial for output.
  SerialPort.println("Starting...");
  pinMode(enc_pin1, INPUT);
  pinMode(enc_pin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_pin1), interruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_pin2), interruptB, CHANGE);
  
  Dabble.begin("Toy Robot");

  DEV_I2C.begin(); // Initialize I2C bus. 
  u8g2.begin(); //Initialize OLED
  randomSeed(analogRead(0)); // Seed the random number generator
  lastBlinkTime = millis(); // Initialize lastBlinkTime
  lastMovementSequenceTime = millis(); // Initialize the last movement time
  Serial.println("Setup Complete");
}

void loop() {
  controllerLogic(); 
  eyeLogic();
}

void controllerLogic() {
  Dabble.processInput();
  unsigned long currentMillis = millis();

  // Check gamepad buttons in priority order.
  if (GamePad.isUpPressed())
  {
    if (currentCommand != FORWARD) {
      currentCommand = FORWARD;
      commandStartTime = currentMillis;
      ticksA = 0;
      ticksB = 0;
    }
    if(currentMillis - commandStartTime >= debounceDelay) {
      Serial.println("Forward.");
      moveMotor1Forward();
      motor2.drive(200);
    }
  }
  else if (GamePad.isDownPressed())
  {
    if (currentCommand != BACKWARD) {
      currentCommand = BACKWARD;
      commandStartTime = currentMillis;
      ticksA = 0;
      ticksB = 0;
    }
    if(currentMillis - commandStartTime >= debounceDelay) {
      Serial.println("Backward.");
      moveMotor1Backward();
      motor2.drive(-200);
    }
  }
  else if (GamePad.isLeftPressed())
  {
    if (currentCommand != LEFT_CMD) {
      currentCommand = LEFT_CMD;
      commandStartTime = currentMillis;
      ticksA = 0;
      ticksB = 0;
    }
    if(currentMillis - commandStartTime >= debounceDelay) {
      Serial.println("Left.");
      moveMotor1Forward();
      motor2.drive(-200);
    }
  }
  else if (GamePad.isRightPressed())
  {
    if (currentCommand != RIGHT_CMD) {
      currentCommand = RIGHT_CMD;
      commandStartTime = currentMillis;
      ticksA = 0;
      ticksB = 0;
    }
    if(currentMillis - commandStartTime >= debounceDelay) {
      Serial.println("Right.");
      motor2.drive(200);
      moveMotor1Backward();
    }
  }
  else {
    currentCommand = NONE;
    brake(motor1, motor2);
    motor3.brake();
  }
}
void eyeLogic(){
  // Define the desired intervals
  unsigned long movementInterval = random(5000, 10000); 
  unsigned long holdTargetDuration = random(100, 500);
  unsigned long centerWaitDuration = random(2000, 4000); 

  eyeAnimation(movementInterval, holdTargetDuration, centerWaitDuration);
}

void interruptA(){
  ticksA++;
}

void interruptB(){
  ticksB++;
}

void moveMotor1Forward(){
  int target = ticksB;
  float kp = 0.7; // Tune as needed
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);
  float speed = 200 + u;
  if (speed > 255){
    speed = 255;
  }
  if (speed < 0) {
    speed = 0;
  }
  motor1.drive(speed);
  Serial.print(u);
  Serial.print("\t");
  Serial.print((float)ticksA - ticksB);
  Serial.print("\t");
  Serial.print(speed);
  Serial.println();
}

void moveMotor1Backward(){
  int target = ticksB;
  float kp = 0.7; // Tune as needed
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);
  float speed = (200 + u) * -1;
  if (speed < -255){
    speed = -255;
  }
  if (speed > 0) {
    speed = 0;
  }
  motor1.drive(speed);
  Serial.print(u);
  Serial.print("\t");
  Serial.print((float)ticksA - ticksB);
  Serial.print("\t");
  Serial.print(200 + u);
  Serial.println();
}

float pidController(int target, float kp, float kd, float ki) {
  // Use esp_timer_get_time() to get microseconds and convert to seconds.
  long currT = esp_timer_get_time();
  float deltaT = ((float)currT - prevT)/1e6;

  int e = target - ticksA;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  prevT = currT;
  ePrevious = e;

  return u;
}

// Function to move both eyes towards a target, returns true when target is reached
bool moveEyesToTarget(int x_target, int y_target) {
    bool reached = true;
    int dx_left = 0;
    int dy_left = 0;

    if (left_eye.y > y_target) {
        dy_left = -movementStep;
        reached = false;
    } else if (left_eye.y < y_target) {
        dy_left = movementStep;
        reached = false;
    }

    if (left_eye.x > x_target) {
        dx_left = -movementStep;
        reached = false;
    } else if (left_eye.x < x_target) {
        dx_left = movementStep;
        reached = false;
    }

    left_eye.move(dx_left, dy_left);
    right_eye.move(dx_left, dy_left); // Keep movements synchronized

    return reached; // Return true if the target has been reached
}

// Function to return eyes to the initial position, returns true when initial position is reached
bool returnEyesFromTarget() {
    bool reached = true;
    int dx_left = 0;
    int dy_left = 0;

    if (left_eye.y > left_eye.initial_y) {
        dy_left = -movementStep;
        reached = false;
    } else if (left_eye.y < left_eye.initial_y) {
        dy_left = movementStep;
        reached = false;
    }

    if (left_eye.x > left_eye.initial_x) {
        dx_left = -movementStep;
        reached = false;
    } else if (left_eye.x < left_eye.initial_x) {
        dx_left = movementStep;
        reached = false;
    }

    left_eye.move(dx_left, dy_left);
    right_eye.move(dx_left, dy_left);

    return reached; // Return true if the initial position has been reached
}

void checkSleep(){
  unsigned long currentMillis = millis();
  if (currentMillis - lastButtonPress >= IDLE_TIME_LIMIT && currentEyeState != EyeState::SLEEPING) {
    currentEyeState = EyeState::SLEEPING;
    Serial.println("Robot is going to sleep!");
  } 
}

// Function to encapsulate the eye animation logic
void eyeAnimation(unsigned long MOVEMENT_INTERVAL, unsigned long HOLD_TARGET_DURATION, unsigned long CENTER_WAIT) {
    unsigned long currentMillis = millis();

    // Check for blink only if NOT sleeping
    if (currentEyeState != EyeState::SLEEPING && currentMillis - lastBlinkTime >= BLINK_INTERVAL && currentEyeState != EyeState::CLOSE_EYES && currentEyeState != EyeState::OPEN_EYES) {
        previousEyeState = currentEyeState;
        currentEyeState = EyeState::CLOSE_EYES;
        stateStartTime = currentMillis;
        lastBlinkTime = currentMillis;
    }

    u8g2.clearBuffer();
    left_eye.draw(u8g2);
    right_eye.draw(u8g2);
    u8g2.sendBuffer();

    // Serial output for debugging
    String serialOutput = ", Left Eye X: " + String(left_eye.x);
    serialOutput += ", Left Eye Y: " + String(left_eye.y);
    serialOutput += ", Left Eye Height: " + String(left_eye.height);
    serialOutput += ", Right Eye Height: " + String(right_eye.height);
    serialOutput += ", Target X: " + String(target_x);
    serialOutput += ", Target Y: " + String(target_y);
    serialOutput += ", Movement Interval: " + String(MOVEMENT_INTERVAL);
    serialOutput += " State: ";
    if (currentEyeState == EyeState::CENTER_WAIT) serialOutput += "CENTER_WAIT";
    else if (currentEyeState == EyeState::MOVE_TO_TARGET) serialOutput += "MOVE_TO_TARGET";
    else if (currentEyeState == EyeState::PAUSE_MOVEMENT) serialOutput += "PAUSE_MOVEMENT";
    else if (currentEyeState == EyeState::HOLD_TARGET) serialOutput += "HOLD_TARGET";
    else if (currentEyeState == EyeState::RETURN_TO_CENTER) serialOutput += "RETURN_TO_CENTER";
    else if (currentEyeState == EyeState::CLOSE_EYES) serialOutput += "CLOSE_EYES";
    else if (currentEyeState == EyeState::OPEN_EYES) serialOutput += "OPEN_EYES";
    else if (currentEyeState == EyeState::WAKING_UP) serialOutput += "WAKING_UP";
    //Serial.println(serialOutput);

    switch (currentEyeState) {
        case EyeState::WAKING_UP:
          left_eye.graduallyOpen(2);
          right_eye.graduallyOpen(2);
          Serial.println("Robot is waking up!");
          if (left_eye.height >= left_eye.initial_height) {
            lastMovementSequenceTime = millis();
            currentEyeState = EyeState::CENTER_WAIT;
            lastButtonPress = millis();
            stateStartTime = currentMillis;
          }
          break;
        case EyeState::SLEEPING:
          left_eye.graduallyClose(2);
          right_eye.graduallyClose(2);
          Serial.println("Robot is sleeping!");
          // Add logic to wake up after a certain time
          if (left_eye.height <= 1) { // Transition only when eyes are fully closed
              // Add logic to wake up after a certain time
              if (currentMillis - stateStartTime >= 5000) { // Example: wake up after 5 seconds
                  currentEyeState = EyeState::WAKING_UP; // Or transition to a waking up state
                  stateStartTime = currentMillis;
                  Serial.println("Eyes closed, transitioning to WAKING_UP"); // Debugging
              }
          }
          break; // Important: Add break here!
        case EyeState::CENTER_WAIT:
          if (currentMillis - lastMovementSequenceTime >= MOVEMENT_INTERVAL) {
              currentEyeState = EyeState::MOVE_TO_TARGET;
              stateStartTime = currentMillis; // Reset timer for the next state
              // Serial.println("Transitioning to MOVE_TO_TARGET");
              // Generate new random target position for the left eye
              target_x = random(x_min, x_max + 1); // +1 because the upper bound is exclusive
              target_y = random(y_min, y_max + 1);
              // Serial.print("New Target X: "); Serial.print(target_x);
              // Serial.print(", New Target Y: "); Serial.println(target_y);
              checkSleep();
          }
          break;
        case EyeState::MOVE_TO_TARGET:
          if (random(0, 100) < 5) {
              currentEyeState = EyeState::PAUSE_MOVEMENT;
              pauseMovementStartTime = currentMillis;
              pauseMovementDuration = random(100, 200); // Random time between 500 and 1000 ms
              // Serial.print("Transitioning to PAUSE_MOVEMENT for: "); Serial.print(pauseMovementDuration); Serial.println("ms");
          } else if (moveEyesToTarget(target_x, target_y)) {
              // Eyes have reached the target
              currentEyeState = EyeState::HOLD_TARGET;
              stateStartTime = currentMillis; // Reset timer for the next state
              // Serial.println("Transitioning to HOLD_TARGET");
          }
          break;
        case EyeState::PAUSE_MOVEMENT:
          if (currentMillis - pauseMovementStartTime >= pauseMovementDuration) {
              currentEyeState = EyeState::MOVE_TO_TARGET; // Continue moving to the target
              // Serial.println("Transitioning back to MOVE_TO_TARGET after pause");
          }
          break;
        case EyeState::HOLD_TARGET:
          if (currentMillis - stateStartTime >= HOLD_TARGET_DURATION) {
              currentEyeState = EyeState::RETURN_TO_CENTER;
              stateStartTime = currentMillis; // Reset timer for the next state
              // Serial.println("Transitioning to RETURN_TO_CENTER");
          }
          break;
        case EyeState::RETURN_TO_CENTER:
          if (returnEyesFromTarget()) {
              currentEyeState = EyeState::CENTER_WAIT;
              stateStartTime = currentMillis; // Reset timer for the next state
              lastMovementSequenceTime = currentMillis; // Update the last movement time
              // Serial.println("Transitioning to CENTER_WAIT");
          }
          break;
        case EyeState::CLOSE_EYES:
          left_eye.graduallyClose(20);
          right_eye.graduallyClose(20);
          if (left_eye.height <= 1) {
              currentEyeState = EyeState::OPEN_EYES;
              stateStartTime = currentMillis;
              // Serial.println("Transitioning to OPEN_EYES");
          }
          break;
        case EyeState::OPEN_EYES:
          left_eye.graduallyOpen(20);
          right_eye.graduallyOpen(20);
          if (left_eye.height >= left_eye.initial_height) {
              // After blinking, return to the state we were in before blinking
              currentEyeState = previousEyeState; // Go back to the previous state
              stateStartTime = currentMillis;
              // Serial.print("Transitioning back to: ");
              // if (currentEyeState == EyeState::CENTER_WAIT) Serial.println("CENTER_WAIT");
              // else if (currentEyeState == EyeState::MOVE_TO_TARGET) Serial.println("MOVE_TO_TARGET");
              // else if (currentEyeState == EyeState::HOLD_TARGET) Serial.println("HOLD_TARGET");
              // else if (currentEyeState == EyeState::RETURN_TO_CENTER) Serial.println("RETURN_TO_CENTER");
              // Serial.println();
          }
          break;
    }
}

unsigned long getRndBiasedRange(unsigned long minPrimary, unsigned long maxPrimary,
unsigned long minSecondary, unsigned long maxSecondary, float probabilityPrimary) {
  float chance = random(0, 1000) / 1000.0;
  if (chance < probabilityPrimary) {
    return random(minPrimary, maxPrimary + 1);
  } else {
    return random(minSecondary, maxSecondary + 1);
  }
}
