#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ  50

// Head Servo Configuration (270 degree servos)
#define HEAD_SERVO_MIN_US     500
#define HEAD_SERVO_MAX_US     2500
#define HEAD_SERVO_RANGE_DEG  270

//Jaw Servo Configuration (180 degree servo)
#define JAW_MIN_PWM     150   
#define JAW_MAX_PWM     600   
#define JAW_RANGE_DEG   180   

// === Servo Channels ===
#define SERVO_NECK_YAW    15
#define SERVO_NECK_LEFT   14
#define SERVO_NECK_RIGHT  13
#define JAW_CHANNEL       6

//Neutral/Home Positions 
#define ANGLE_YAW     120  // Home position for straight forward
#define ANGLE_LEFT    170  // Home position 
#define ANGLE_RIGHT   110  // Home position 
#define JAW_CLOSED_ANGLE  96   // Mouth closed position
#define JAW_OPEN_ANGLE    35   // Mouth fully open position

// Safety Limits 
#define MAX_MOVEMENT  20  
#define YAW_MIN       80  
#define YAW_MAX       160  
#define LEFT_MIN      150  
#define LEFT_MAX      190  
#define RIGHT_MIN     90   
#define RIGHT_MAX     130  

// Smoothing Parameters 
#define SMOOTH_STEPS   50    // Number of interpolation steps
#define SMOOTH_DELAY   20    // Delay between steps (ms)
#define ACCEL_FACTOR   2.5   // Exponential acceleration factor

// Current Position Tracking
int currentYawAngle = ANGLE_YAW;
int currentLeftAngle = ANGLE_LEFT;
int currentRightAngle = ANGLE_RIGHT;
int currentJawAngle = JAW_CLOSED_ANGLE;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Initializing smooth combined head and jaw control...");
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  // Initialize to home positions
  centerHead();
  closeJaw();
  
  Serial.println("Smooth servo control ready!");
  Serial.println("----Available Commands----");
  Serial.println("Basic Movement:");
  Serial.println("  c - Center head and close jaw");
  Serial.println("  l - Look left");
  Serial.println("  r - Look right");
  Serial.println("  u - Look up");
  Serial.println("  d - Look down");
  Serial.println("  o - Open jaw");
  Serial.println("  j - Close jaw");
  Serial.println("Expression Commands:");
  Serial.println("  h - Happy expression");
  Serial.println("  s - Surprised expression");
  Serial.println("  t - Talk sequence");
  Serial.println("  n - Nodding motion");
  Serial.println("  k - Head shake with jaw");
  Serial.println("  f - Confused expression");
  Serial.println("  a - Attentive listening");
  Serial.println("Test Commands:");
  Serial.println("  z - Test all movements");
  Serial.println("  p - Print current positions");
}
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while(Serial.available()) Serial.read(); // Clear buffer
    
    switch(cmd) {
      case 'c': centerHead(); closeJaw(); break;
      case 'l': lookLeft(15); break;
      case 'r': lookRight(15); break;
      case 'u': lookUp(10); break;
      case 'd': lookDown(10); break;
      case 'o': openJaw(); break;
      case 'j': closeJaw(); break;
      case 'h': expressionHappy(); break;
      case 's': expressionSurprised(); break;
      case 't': talkSequence(); break;
      case 'n': noddingMotion(); break;
      case 'k': headShakeWithJaw(); break;
      case 'f': confusedExpression(); break;
      case 'a': attentiveListening(); break;
      case 'z': testAllMovements(); break;
      case 'p': printCurrentPositions(); break;
      default:
        Serial.println("Available: c,l,r,u,d,o,j,h,s,t,n,k,f,a,z,p");
        break;
    }
  }
}

// Smooth Movement Function
void moveServoSmooth(int channel, int startAngle, int endAngle) {
  if (startAngle == endAngle) return;
  
  Serial.print("Smooth move Ch");
  Serial.print(channel);
  Serial.print(": ");
  Serial.print(startAngle);
  Serial.print("->");
  Serial.println(endAngle);
  
  for (int step = 0; step <= SMOOTH_STEPS; step++) {
    // normalized position (0.0 to 1.0)
    float t = (float)step / SMOOTH_STEPS;
    // Aexponential ease-in-out curve
    float smoothT;
    if (t < 0.5) {
      // ease-in phase 
      smoothT = pow(2 * t, ACCEL_FACTOR) / 2;
    } else {
      // ease-out phase 
      smoothT = 1 - pow(2 * (1 - t), ACCEL_FACTOR) / 2;
    }
    
    // Interpolate betn start and end 
    int currentAngle = startAngle + (int)((endAngle - startAngle) * smoothT);
    if (channel == JAW_CHANNEL) {
      setJawPositionDirect(currentAngle);
    } else {
      setHeadServo(channel, currentAngle);
    }
    delay(SMOOTH_DELAY);
  }

  // Update position tracking
  switch(channel) {
    case SERVO_NECK_YAW: currentYawAngle = endAngle; break;
    case SERVO_NECK_LEFT: currentLeftAngle = endAngle; break;
    case SERVO_NECK_RIGHT: currentRightAngle = endAngle; break;
    case JAW_CHANNEL: currentJawAngle = endAngle; break;
  }
}
// Multiple Servo Smooth Movement
void moveMultipleServosSmooth(int channels[], int startAngles[], int endAngles[], int numServos) {
  Serial.print("Multi-servo smooth move: ");
  for (int i = 0; i < numServos; i++) {
    Serial.print("Ch");
    Serial.print(channels[i]);
    Serial.print(":");
    Serial.print(startAngles[i]);
    Serial.print("->");
    Serial.print(endAngles[i]);
    if (i < numServos - 1) Serial.print(", ");
  }
  Serial.println();
  
  for (int step = 0; step <= SMOOTH_STEPS; step++) {
    float t = (float)step / SMOOTH_STEPS;
    
    // exponential ease-in-out curve
    float smoothT;
    if (t < 0.5) {
      smoothT = pow(2 * t, ACCEL_FACTOR) / 2;
    } else {
      smoothT = 1 - pow(2 * (1 - t), ACCEL_FACTOR) / 2;
    }
    
    // Move all servos simultaneously
    for (int i = 0; i < numServos; i++) {
      int currentAngle = startAngles[i] + (int)((endAngles[i] - startAngles[i]) * smoothT);
      
      if (channels[i] == JAW_CHANNEL) {
        setJawPositionDirect(currentAngle);
      } else {
        setHeadServo(channels[i], currentAngle);
      }
    }
    
    delay(SMOOTH_DELAY);
  }
  
  // Update position tracking
  for (int i = 0; i < numServos; i++) {
    switch(channels[i]) {
      case SERVO_NECK_YAW: currentYawAngle = endAngles[i]; break;
      case SERVO_NECK_LEFT: currentLeftAngle = endAngles[i]; break;
      case SERVO_NECK_RIGHT: currentRightAngle = endAngles[i]; break;
      case JAW_CHANNEL: currentJawAngle = endAngles[i]; break;
    }
  }
}

//Head Servo Control (270 degree servos)
void setHeadServo(int channel, int angle) {
  // channel-specific limits
  switch(channel) {
    case SERVO_NECK_YAW:
      angle = constrain(angle, YAW_MIN, YAW_MAX);
      break;
    case SERVO_NECK_LEFT:
      angle = constrain(angle, LEFT_MIN, LEFT_MAX);
      break;
    case SERVO_NECK_RIGHT:
      angle = constrain(angle, RIGHT_MIN, RIGHT_MAX);
      break;
  }
  
  // overall range constraint
  angle = constrain(angle, 0, HEAD_SERVO_RANGE_DEG);
  
  // Convert to microseconds and then to PWM value
  int pulse_us = map(angle, 0, HEAD_SERVO_RANGE_DEG, HEAD_SERVO_MIN_US, HEAD_SERVO_MAX_US);
  float pulseLength = 1000000.0 / (SERVO_FREQ * 4096.0);
  int pwmVal = int(pulse_us / pulseLength);
  pwm.setPWM(channel, 0, pwmVal);
}

// Jaw Servo Control
void setJawPositionDirect(int angle) {
  angle = constrain(angle, 0, JAW_RANGE_DEG);
  
  // Map angle directly to PWM value
  int pwmVal = map(angle, 0, JAW_RANGE_DEG, JAW_MIN_PWM, JAW_MAX_PWM);
  pwm.setPWM(JAW_CHANNEL, 0, pwmVal);
}

void setJawPosition(int angle) {
  moveServoSmooth(JAW_CHANNEL, currentJawAngle, angle);
  Serial.println("Jaw smoothly moved to " + String(angle) + " degrees");
}

//Head Movement Functions
void lookRight(int degrees) {
  degrees = constrain(degrees, 0, MAX_MOVEMENT);
  int new_angle = ANGLE_YAW - degrees;
  moveServoSmooth(SERVO_NECK_YAW, currentYawAngle, new_angle);
  Serial.println("Looking right " + String(degrees) + " degrees (angle: " + String(new_angle) + ")");
}
void lookLeft(int degrees) {
  degrees = constrain(degrees, 0, MAX_MOVEMENT);
  int new_angle = ANGLE_YAW + degrees;
  moveServoSmooth(SERVO_NECK_YAW, currentYawAngle, new_angle);
  Serial.println("Looking left " + String(degrees) + " degrees (angle: " + String(new_angle) + ")");
}
void lookUp(int degrees) {
  degrees = constrain(degrees, 0, MAX_MOVEMENT);
  int left_angle = ANGLE_LEFT + degrees;
  int right_angle = ANGLE_RIGHT - degrees;
  int channels[2] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT};
  int startAngles[2] = {currentLeftAngle, currentRightAngle};
  int endAngles[2] = {left_angle, right_angle};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 2);
  Serial.println("Looking up " + String(degrees) + " degrees (L:" + String(left_angle) + " R:" + String(right_angle) + ")");
}
void lookDown(int degrees) {
  degrees = constrain(degrees, 0, MAX_MOVEMENT);
  int left_angle = ANGLE_LEFT - degrees;
  int right_angle = ANGLE_RIGHT + degrees;
  int channels[2] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT};
  int startAngles[2] = {currentLeftAngle, currentRightAngle};
  int endAngles[2] = {left_angle, right_angle};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 2);
  Serial.println("Looking down " + String(degrees) + " degrees (L:" + String(left_angle) + " R:" + String(right_angle) + ")");
}
void centerHead() {
  int channels[3] = {SERVO_NECK_YAW, SERVO_NECK_LEFT, SERVO_NECK_RIGHT};
  int startAngles[3] = {currentYawAngle, currentLeftAngle, currentRightAngle};
  int endAngles[3] = {ANGLE_YAW, ANGLE_LEFT, ANGLE_RIGHT};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 3);
  Serial.println("Head smoothly centered");
}
//Jaw Movement Functions 
void openJaw() {
  setJawPosition(JAW_OPEN_ANGLE);
  Serial.println("Jaw smoothly opened");
}
void closeJaw() {
  setJawPosition(JAW_CLOSED_ANGLE);
  Serial.println("Jaw smoothly closed");
}

// Combined Movement Functions
void moveHead(int yaw_offset, int pitch_offset) {
  yaw_offset = constrain(yaw_offset, -MAX_MOVEMENT, MAX_MOVEMENT);
  pitch_offset = constrain(pitch_offset, -MAX_MOVEMENT, MAX_MOVEMENT);
  int yaw_angle = ANGLE_YAW + yaw_offset;
  int left_angle = ANGLE_LEFT + pitch_offset;
  int right_angle = ANGLE_RIGHT - pitch_offset;
  int channels[3] = {SERVO_NECK_YAW, SERVO_NECK_LEFT, SERVO_NECK_RIGHT};
  int startAngles[3] = {currentYawAngle, currentLeftAngle, currentRightAngle};
  int endAngles[3] = {yaw_angle, left_angle, right_angle};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 3);
  Serial.println("Moving head smoothly: yaw=" + String(yaw_offset) + ", pitch=" + String(pitch_offset));
}

//Expression Functions
void talkSequence() {
  Serial.println("Smooth talking sequence...");
  for(int i = 0; i < 3; i++) {
    setJawPosition(JAW_OPEN_ANGLE);
    delay(300);
    setJawPosition(JAW_CLOSED_ANGLE);
    delay(200);
  }
}
void expressionHappy() {
  Serial.println("Smooth happy expression...");
  // head up and jaw open
  int channels[3] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT, JAW_CHANNEL};
  int startAngles[3] = {currentLeftAngle, currentRightAngle, currentJawAngle};
  int endAngles[3] = {ANGLE_LEFT + 5, ANGLE_RIGHT - 5, JAW_OPEN_ANGLE};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 3);
  delay(500);
  // Close jaw and return to center
  int channels2[3] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT, JAW_CHANNEL};
  int startAngles2[3] = {currentLeftAngle, currentRightAngle, currentJawAngle};
  int endAngles2[3] = {ANGLE_LEFT, ANGLE_RIGHT, JAW_CLOSED_ANGLE};
  moveMultipleServosSmooth(channels2, startAngles2, endAngles2, 3);
}
void expressionSurprised() {
  Serial.println("Smooth surprised expression...");
  // Quick head up and jaw open
  int channels[3] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT, JAW_CHANNEL};
  int startAngles[3] = {currentLeftAngle, currentRightAngle, currentJawAngle};
  int endAngles[3] = {ANGLE_LEFT + 8, ANGLE_RIGHT - 8, JAW_OPEN_ANGLE};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 3);
  delay(1000);
  // Return to center
  int channels2[3] = {SERVO_NECK_LEFT, SERVO_NECK_RIGHT, JAW_CHANNEL};
  int startAngles2[3] = {currentLeftAngle, currentRightAngle, currentJawAngle};
  int endAngles2[3] = {ANGLE_LEFT, ANGLE_RIGHT, JAW_CLOSED_ANGLE};
  moveMultipleServosSmooth(channels2, startAngles2, endAngles2, 3);
}
void noddingMotion() {
  Serial.println("Smooth nodding...");
  for(int i = 0; i < 2; i++) {
    lookDown(15);
    delay(400);
    lookUp(15);
    delay(400);
  }
  centerHead();
}
void headShakeWithJaw() {
  Serial.println("Smooth head shake with jaw...");
  for(int i = 0; i < 2; i++) {
    // Left with jaw open
    int channels[2] = {SERVO_NECK_YAW, JAW_CHANNEL};
    int startAngles[2] = {currentYawAngle, currentJawAngle};
    int endAngles[2] = {ANGLE_YAW + 12, JAW_OPEN_ANGLE};
    moveMultipleServosSmooth(channels, startAngles, endAngles, 2);
    delay(300);
    // Right with jaw closed
    int startAngles2[2] = {currentYawAngle, currentJawAngle};
    int endAngles2[2] = {ANGLE_YAW - 12, JAW_CLOSED_ANGLE};
    moveMultipleServosSmooth(channels, startAngles2, endAngles2, 2);
    delay(300);
  }
  // Return to center
  int channels[2] = {SERVO_NECK_YAW, JAW_CHANNEL};
  int startAngles[2] = {currentYawAngle, currentJawAngle};
  int endAngles[2] = {ANGLE_YAW, JAW_CLOSED_ANGLE};
  moveMultipleServosSmooth(channels, startAngles, endAngles, 2);
}
void confusedExpression() {
  Serial.println("Smooth confused expression...");
  lookLeft(15);
  delay(300);
  lookRight(20);
  delay(300);
  lookLeft(15);
  delay(200);
  setJawPosition(70); // Slightly open
  delay(500);
  closeJaw();
  centerHead();
}
void attentiveListening() {
  Serial.println("Smooth attentive listening...");
  lookUp(3);
  delay(200);
  for(int i = 0; i < 3; i++) {
    lookLeft(5);
    delay(400);
    lookRight(10);
    delay(400);
    lookLeft(5);
    delay(400);
  }
  centerHead();
}

// Utility Functions 
void printCurrentPositions() {
  Serial.println("Current Positions");
  Serial.println("Head - Yaw: " + String(currentYawAngle) + ", Left: " + String(currentLeftAngle) + ", Right: " + String(currentRightAngle));
  Serial.println("Jaw: " + String(currentJawAngle) + " degrees");
  Serial.println("Target Positions");
  Serial.println("Head - Yaw: " + String(ANGLE_YAW) + ", Left: " + String(ANGLE_LEFT) + ", Right: " + String(ANGLE_RIGHT));
  Serial.println("Jaw - Closed: " + String(JAW_CLOSED_ANGLE) + ", Open: " + String(JAW_OPEN_ANGLE));
  Serial.println("Safety Limits");
  Serial.println("Yaw: " + String(YAW_MIN) + "-" + String(YAW_MAX));
  Serial.println("Left: " + String(LEFT_MIN) + "-" + String(LEFT_MAX));
  Serial.println("Right: " + String(RIGHT_MIN) + "-" + String(RIGHT_MAX));
  Serial.println("Max Movement: " + String(MAX_MOVEMENT) + " degrees");
  Serial.println("Smoothing: " + String(SMOOTH_STEPS) + " steps, " + String(SMOOTH_DELAY) + "ms delay");
}

// Test Functions 
void testAllMovements() {
  Serial.println("Testing All Smooth Movements");
  // Test head movements
  Serial.println("Testing smooth head movements...");
  centerHead();
  delay(1000);
  lookLeft(15);
  delay(1000);
  lookRight(15);
  delay(1000);
  lookUp(10);
  delay(1000);
  lookDown(10);
  delay(1000);
  centerHead();
  // Test jaw movements
  Serial.println("Testing smooth jaw movements...");
  closeJaw();
  delay(1000);
  openJaw();
  delay(1000);
  closeJaw();
  // Test combined movements
  Serial.println("Testing smooth combined movements...");
  expressionHappy();
  delay(1000);
  expressionSurprised();
  delay(1000);
  Serial.println("All Smooth Tests Complete");
}
