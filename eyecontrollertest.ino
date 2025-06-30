#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN        150
#define SERVO_MAX        600
#define SERVO_FREQ       50

// Servo channels
#define SERVO_LOOK_LR    0
#define SERVO_LOOK_UD    1
#define SERVO_LID_BL     5
#define SERVO_LID_TL     4
#define SERVO_LID_BR     3
#define SERVO_LID_TR     2

//center positions (accounting for direction reversal)
#define CENTER_EYE_LR    128  
#define CENTER_EYE_UD    100
#define CENTER_LID_BL    90
#define CENTER_LID_TL    90
#define CENTER_LID_BR    90
#define CENTER_LID_TR    90

#define SMALL_MOVEMENT   25
#define TINY_MOVEMENT    5

// Smoothing parameters
#define SMOOTH_STEPS     50    // Number of interpolation steps
#define SMOOTH_DELAY     20    // Delay between steps (ms)
#define ACCEL_FACTOR     2.5   // Exponential acceleration factor

// Idle motion parameters
#define IDLE_UPDATE_INTERVAL  150   // Base update interval for idle motion (ms)
#define MICROSACCADE_CHANCE   25    // Percentage chance of microsaccade per update
#define BLINK_CHANCE          3     // Percentage chance of blink per update
#define DRIFT_CHANCE          15    // Percentage chance of position drift per update
#define DRAMATIC_MOVE_CHANCE  8     // Percentage chance of dramatic eye movement per update
#define MAX_IDLE_OFFSET       25    // Maximum offset from center during idle
#define MICROSACCADE_SIZE     3     // Size of microsaccade movements
#define BLINK_DURATION        300   // How long blinks last (ms)
#define DRAMATIC_MOVE_RANGE   35    // Range for dramatic movements (degrees)

// Adjustable smoothing parameters for dramatic moves
#define DRAMATIC_SMOOTH_STEPS_MIN   30    // Minimum smoothing steps
#define DRAMATIC_SMOOTH_STEPS_MAX   60    // Maximum smoothing steps
#define DRAMATIC_SMOOTH_DELAY_MIN   15    // Minimum delay between steps (ms)
#define DRAMATIC_SMOOTH_DELAY_MAX   25    // Maximum delay between steps (ms)
#define DRAMATIC_ACCEL_FACTOR       2.8   // Exponential acceleration factor for dramatic moves


void setup() {
  Serial.begin(9600);
  while (!Serial){
    delay(10);
  }
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  Serial.println(F("Setting centers..."));
  setAllToCenter();
  delay(100);

  Serial.println(F("Commands: 1-6, a, c, f, s, t, i, o"));
}
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while(Serial.available()) Serial.read();
    
    switch(cmd) {
      case '1': testEyeLR(); break;
      case '2': testEyeUD(); break;
      case '3': testEyelid(SERVO_LID_BL, CENTER_LID_BL, F("BL")); break;
      case '4': testEyelid(SERVO_LID_TL, CENTER_LID_TL, F("TL")); break;
      case '5': testEyelid(SERVO_LID_BR, CENTER_LID_BR, F("BR")); break;
      case '6': testEyelid(SERVO_LID_TR, CENTER_LID_TR, F("TR")); break;
      case 'a': testAll(); break;
      case 'c': setAllToCenter(); break;
      case 't': tinyTest(); break;
      case 'f': finalMovementTest(); break;
      case 's': smoothTest(); break;
      case 'o': circularLookAround(); break;
      case 'i': idleMotion(); break;
    }
  }
}

void setServoRaw(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

void setServoCorrected(int channel, int angle) {
  int finalAngle = angle;

   if (channel == SERVO_LOOK_LR ||    // Eye LR - reversed
      channel == SERVO_LID_BL ||     // Bottom Left - reversed  
      channel == SERVO_LID_TR) {     // Top Right - reversed
    finalAngle = 180 - angle;
  }

  setServoRaw(channel, finalAngle);

  Serial.print(F("Ch"));
  Serial. print(channel);
  Serial.print(F(":"));
  Serial.print(angle);
  if (finalAngle != angle){
    Serial.print(F("->"));
    Serial.print(finalAngle);
    Serial.print(F(" (actual:"));
    Serial.print(finalAngle);
    Serial.print(F("°)"));
  }
  Serial.println();
}

// Smooth movement function with exponential acceleration/deceleration
void moveServoSmooth(int channel, int startAngle, int endAngle) {
  if (startAngle == endAngle) return;
  
  Serial.print(F("Smooth move Ch"));
  Serial.print(channel);
  Serial.print(F(": "));
  Serial.print(startAngle);
  Serial.print(F("->"));
  Serial.println(endAngle);
  
  for (int step = 0; step <= SMOOTH_STEPS; step++) {
    // Calculate normalized position (0.0 to 1.0)
    float t = (float)step / SMOOTH_STEPS;
    
    // Apply exponential ease-in-out curve
    float smoothT;
    if (t < 0.5) {
      // Acceleration phase (first half)
      smoothT = pow(2 * t, ACCEL_FACTOR) / 2;
    } else {
      // Deceleration phase (second half)
      smoothT = 1 - pow(2 * (1 - t), ACCEL_FACTOR) / 2;
    }
    
    // Interpolate between start and end angles
    int currentAngle = startAngle + (int)((endAngle - startAngle) * smoothT);
    
    setServoCorrected(channel, currentAngle);
    
    delay(SMOOTH_DELAY);
  }
}

int getCurrentAngle(int channel) {
  // For now, we'll assume servos are at their center positions
  switch(channel) {
    case SERVO_LOOK_LR: return CENTER_EYE_LR;
    case SERVO_LOOK_UD: return CENTER_EYE_UD;
    case SERVO_LID_BL: return CENTER_LID_BL;
    case SERVO_LID_TL: return CENTER_LID_TL;
    case SERVO_LID_BR: return CENTER_LID_BR;
    case SERVO_LID_TR: return CENTER_LID_TR;
    default: return 90;
  }
}

void moveMultipleServos(int channels[], int startAngles[], int endAngles[], int numServos) {
  Serial.print(F("Multi-servo move: "));
  for (int i = 0; i < numServos; i++) {
    Serial.print(F("Ch"));
    Serial.print(channels[i]);
    Serial.print(F(":"));
    Serial.print(startAngles[i]);
    Serial.print(F("->"));
    Serial.print(endAngles[i]);
    if (i < numServos - 1) Serial.print(F(", "));
  }
  Serial.println();
  
  for (int step = 0; step <= SMOOTH_STEPS; step++) {
    float t = (float)step / SMOOTH_STEPS;
    
    // Apply exponential ease-in-out curve
    float smoothT;
    if (t < 0.5) {
      smoothT = pow(2 * t, ACCEL_FACTOR) / 2;
    } else {
      smoothT = 1 - pow(2 * (1 - t), ACCEL_FACTOR) / 2;
    }
    
    // Move all servos simultaneously
    for (int i = 0; i < numServos; i++) {
      int currentAngle = startAngles[i] + (int)((endAngles[i] - startAngles[i]) * smoothT);
      setServoCorrected(channels[i], currentAngle);
    }
    
    delay(SMOOTH_DELAY);
  }
}

void setAllToCenter() {
  Serial.println(F("Centering (corrected)..."));
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR);
  setServoCorrected(SERVO_LOOK_UD, CENTER_EYE_UD);
  setServoCorrected(SERVO_LID_BL, CENTER_LID_BL);
  setServoCorrected(SERVO_LID_TL, CENTER_LID_TL);
  setServoCorrected(SERVO_LID_BR, CENTER_LID_BR);
  setServoCorrected(SERVO_LID_TR, CENTER_LID_TR);
  delay(500);
}

void testEyeLR() {
  Serial.println(F("Testing Eye LR (CORRECTED + SMOOTH)"));
  Serial.println(F("Should achieve 52° center, 37° right, 67° left"));
  
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR);
  delay(1000);
  
  Serial.println(F("SMOOTH RIGHT..."));
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR, CENTER_EYE_LR + SMALL_MOVEMENT);
  delay(1000);
  
  Serial.println(F("SMOOTH Center..."));
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR + SMALL_MOVEMENT, CENTER_EYE_LR);
  delay(1000);
  
  Serial.println(F("SMOOTH LEFT..."));
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR, CENTER_EYE_LR - SMALL_MOVEMENT);
  delay(1000);
  
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR - SMALL_MOVEMENT, CENTER_EYE_LR);
  Serial.println(F("Smooth movement complete!"));
}

void testEyeUD() {
  Serial.println(F("Testing Eye UD (SMOOTH)"));
  
  setServoCorrected(SERVO_LOOK_UD, CENTER_EYE_UD);
  delay(1000);
  
  Serial.println(F("SMOOTH UP..."));
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD, CENTER_EYE_UD + SMALL_MOVEMENT);
  delay(1000);
  
  Serial.println(F("SMOOTH Center..."));
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD + SMALL_MOVEMENT, CENTER_EYE_UD);
  delay(1000);
  
  Serial.println(F("SMOOTH DOWN..."));
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD, CENTER_EYE_UD - SMALL_MOVEMENT);
  delay(1000);
  
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD - SMALL_MOVEMENT, CENTER_EYE_UD);
  Serial.println(F("Smooth movement complete!"));
}

void testEyelid(int servo, int center, const __FlashStringHelper* name) {
  Serial.print(F("Testing Eyelid "));
  Serial.print(name);
  if (servo == SERVO_LID_BL || servo == SERVO_LID_TR) {
    Serial.println(F(" (CORRECTED)"));
  } else {
    Serial.println();
  }
  
  setServoCorrected(servo, center);
  delay(1000);
  
  Serial.println(F("CLOSE..."));
  setServoCorrected(servo, center + SMALL_MOVEMENT);
  delay(2000);
  
  Serial.println(F("Center..."));
  setServoCorrected(servo, center);
  delay(1000);
  
  Serial.println(F("OPEN..."));
  setServoCorrected(servo, center - SMALL_MOVEMENT);
  delay(2000);
  
  setServoCorrected(servo, center);
  Serial.println(F("Should be correct!"));
}

void testAll() {
  Serial.println(F("Testing All (CORRECTED)..."));
  testEyeLR();
  delay(500);
  testEyeUD();
  delay(500);
  testEyelid(SERVO_LID_BL, CENTER_LID_BL, F("BL"));
  delay(500);
  testEyelid(SERVO_LID_TL, CENTER_LID_TL, F("TL"));
  delay(500);
  testEyelid(SERVO_LID_BR, CENTER_LID_BR, F("BR"));
  delay(500);
  testEyelid(SERVO_LID_TR, CENTER_LID_TR, F("TR"));
  Serial.println(F("All should be correct now!"));
}

void tinyTest() {
  Serial.println(F("Tiny movements (corrected)..."));
  setAllToCenter();
  delay(1000);
  
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR + TINY_MOVEMENT);
  delay(1000);
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR);
  delay(1000);
  
  setServoCorrected(SERVO_LOOK_UD, CENTER_EYE_UD + TINY_MOVEMENT);
  delay(1000);
  setServoCorrected(SERVO_LOOK_UD, CENTER_EYE_UD);
  
  Serial.println(F("Tiny test done"));
}


void finalMovementTest() {
  Serial.println(F("=== FINAL COORDINATED TEST ==="));
  
  // Center everything
  setAllToCenter();
  delay(1000);
  
  // Look right and blink
  Serial.println(F("Look RIGHT + Blink"));
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR + 20);
  delay(500);
  // Close all eyelids
  setServoCorrected(SERVO_LID_BL, CENTER_LID_BL + 20);
  setServoCorrected(SERVO_LID_TL, CENTER_LID_TL + 20);
  setServoCorrected(SERVO_LID_BR, CENTER_LID_BR + 20);
  setServoCorrected(SERVO_LID_TR, CENTER_LID_TR + 20);
  delay(500);
  
  // Open eyes
  setAllToCenter();
  delay(1000);
  
  // Look left and up
  Serial.println(F("Look LEFT + UP"));
  setServoCorrected(SERVO_LOOK_LR, CENTER_EYE_LR - 20);
  setServoCorrected(SERVO_LOOK_UD, CENTER_EYE_UD + 15);
  delay(2000);
  
  // Return to center
  Serial.println(F("Return to center"));
  setAllToCenter();
  delay(1000);
  
  Serial.println(F("Final test complete!"));
  Serial.println(F("All movements should be natural"));
}


void smoothTest() {
  Serial.println(F("=== SMOOTH MOVEMENT DEMO ==="));
  
  // Center everything
  setAllToCenter();
  delay(1000);
  
  Serial.println(F("Smooth eye tracking sequence..."));
  
  // Smooth look sequence
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR, CENTER_EYE_LR + 25); // Right
  delay(500);
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD, CENTER_EYE_UD + 20); // Up
  delay(500);
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR + 25, CENTER_EYE_LR - 25); // Left
  delay(500);
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD + 20, CENTER_EYE_UD - 20); // Down
  delay(500);
  
  // Smooth blink
  Serial.println(F("Smooth blink..."));
  moveServoSmooth(SERVO_LID_BL, CENTER_LID_BL, CENTER_LID_BL + 25);
  moveServoSmooth(SERVO_LID_TL, CENTER_LID_TL, CENTER_LID_TL + 25);
  moveServoSmooth(SERVO_LID_BR, CENTER_LID_BR, CENTER_LID_BR + 25);
  moveServoSmooth(SERVO_LID_TR, CENTER_LID_TR, CENTER_LID_TR + 25);
  delay(200);
  
  // Open eyes smoothly
  moveServoSmooth(SERVO_LID_BL, CENTER_LID_BL + 25, CENTER_LID_BL);
  moveServoSmooth(SERVO_LID_TL, CENTER_LID_TL + 25, CENTER_LID_TL);
  moveServoSmooth(SERVO_LID_BR, CENTER_LID_BR + 25, CENTER_LID_BR);
  moveServoSmooth(SERVO_LID_TR, CENTER_LID_TR + 25, CENTER_LID_TR);
  
  // Return to center smoothly
  Serial.println(F("Return to center smoothly..."));
  moveServoSmooth(SERVO_LOOK_LR, CENTER_EYE_LR - 25, CENTER_EYE_LR);
  moveServoSmooth(SERVO_LOOK_UD, CENTER_EYE_UD - 20, CENTER_EYE_UD);
  
  Serial.println(F("Smooth demo complete!"));
}

void circularLookAround() {
  Serial.println(F("CIRCULAR LOOK AROUND MOTION"));
  Serial.println(F("scanning behavior..."));
  
  // Center everything first
  setAllToCenter();
  delay(1000);
  
  // Define circular motion parameters
  int eyeRadiusLR = 20;  // Horizontal eye movement range
  int eyeRadiusUD = 15;  // Vertical eye movement range
  int lidVariation = 8;  // Eyelid movement variation
  int numPoints = 12;    // Number of points in the circle
  
  Serial.println(F("Starting circular scan..."));
  
  for (int point = 0; point <= numPoints; point++) {
    // Calculate angle for this point (0 to 2π)
    float angle = (float)point * 2.0 * PI / numPoints;
    
    // Calculate eye positions using circular motion
    int eyeLR = CENTER_EYE_LR + (int)(eyeRadiusLR * cos(angle));
    int eyeUD = CENTER_EYE_UD + (int)(eyeRadiusUD * sin(angle));
    
    // Calculate eyelid positions with subtle variations
    int lidBL = CENTER_LID_BL + (int)(lidVariation * 0.3 * sin(angle * 0.8));
    int lidTL = CENTER_LID_TL + (int)(lidVariation * 0.2 * cos(angle * 1.2));
    int lidBR = CENTER_LID_BR + (int)(lidVariation * 0.25 * sin(angle * 1.1));
    int lidTR = CENTER_LID_TR + (int)(lidVariation * 0.3 * cos(angle * 0.9));
    
    // Constrain all values to safe ranges
    eyeLR = constrain(eyeLR, CENTER_EYE_LR - 25, CENTER_EYE_LR + 25);
    eyeUD = constrain(eyeUD, CENTER_EYE_UD - 20, CENTER_EYE_UD + 20);
    lidBL = constrain(lidBL, CENTER_LID_BL - 10, CENTER_LID_BL + 10);
    lidTL = constrain(lidTL, CENTER_LID_TL - 10, CENTER_LID_TL + 10);
    lidBR = constrain(lidBR, CENTER_LID_BR - 10, CENTER_LID_BR + 10);
    lidTR = constrain(lidTR, CENTER_LID_TR - 10, CENTER_LID_TR + 10);
    
    // Prepare arrays for multi-servo movement
    int channels[6] = {SERVO_LOOK_LR, SERVO_LOOK_UD, SERVO_LID_BL, SERVO_LID_TL, SERVO_LID_BR, SERVO_LID_TR};
    int startAngles[6], endAngles[6];
    
    // Get current positions (for first point, use center)
    if (point == 0) {
      startAngles[0] = CENTER_EYE_LR;
      startAngles[1] = CENTER_EYE_UD;
      startAngles[2] = CENTER_LID_BL;
      startAngles[3] = CENTER_LID_TL;
      startAngles[4] = CENTER_LID_BR;
      startAngles[5] = CENTER_LID_TR;
    } else {
      // Calculate previous position
      float prevAngle = (float)(point - 1) * 2.0 * PI / numPoints;
      startAngles[0] = CENTER_EYE_LR + (int)(eyeRadiusLR * cos(prevAngle));
      startAngles[1] = CENTER_EYE_UD + (int)(eyeRadiusUD * sin(prevAngle));
      startAngles[2] = CENTER_LID_BL + (int)(lidVariation * 0.3 * sin(prevAngle * 0.8));
      startAngles[3] = CENTER_LID_TL + (int)(lidVariation * 0.2 * cos(prevAngle * 1.2));
      startAngles[4] = CENTER_LID_BR + (int)(lidVariation * 0.25 * sin(prevAngle * 1.1));
      startAngles[5] = CENTER_LID_TR + (int)(lidVariation * 0.3 * cos(prevAngle * 0.9));
      
      // Constrain start angles too
      startAngles[0] = constrain(startAngles[0], CENTER_EYE_LR - 25, CENTER_EYE_LR + 25);
      startAngles[1] = constrain(startAngles[1], CENTER_EYE_UD - 20, CENTER_EYE_UD + 20);
      startAngles[2] = constrain(startAngles[2], CENTER_LID_BL - 10, CENTER_LID_BL + 10);
      startAngles[3] = constrain(startAngles[3], CENTER_LID_TL - 10, CENTER_LID_TL + 10);
      startAngles[4] = constrain(startAngles[4], CENTER_LID_BR - 10, CENTER_LID_BR + 10);
      startAngles[5] = constrain(startAngles[5], CENTER_LID_TR - 10, CENTER_LID_TR + 10);
    }
    
    // Set target positions
    endAngles[0] = eyeLR;
    endAngles[1] = eyeUD;
    endAngles[2] = lidBL;
    endAngles[3] = lidTL;
    endAngles[4] = lidBR;
    endAngles[5] = lidTR;
    
    Serial.print(F("Point "));
    Serial.print(point);
    Serial.print(F("/"));
    Serial.print(numPoints);
    Serial.print(F(" - Angle: "));
    Serial.print((int)(angle * 180 / PI));
    Serial.println(F("°"));
    
    // Move all servos simultaneously
    moveMultipleServos(channels, startAngles, endAngles, 6);
    
    // Brief pause at each position
    delay(200);
  }
  
  Serial.println(F("Circular scan complete! Returning to center..."));
  
  // Return to center smoothly
  int channels[6] = {SERVO_LOOK_LR, SERVO_LOOK_UD, SERVO_LID_BL, SERVO_LID_TL, SERVO_LID_BR, SERVO_LID_TR};
  int startAngles[6] = {
    CENTER_EYE_LR + (int)(eyeRadiusLR * cos(2.0 * PI)),
    CENTER_EYE_UD + (int)(eyeRadiusUD * sin(2.0 * PI)),
    CENTER_LID_BL + (int)(lidVariation * 0.3 * sin(2.0 * PI * 0.8)),
    CENTER_LID_TL + (int)(lidVariation * 0.2 * cos(2.0 * PI * 1.2)),
    CENTER_LID_BR + (int)(lidVariation * 0.25 * sin(2.0 * PI * 1.1)),
    CENTER_LID_TR + (int)(lidVariation * 0.3 * cos(2.0 * PI * 0.9))
  };
  int endAngles[6] = {CENTER_EYE_LR, CENTER_EYE_UD, CENTER_LID_BL, CENTER_LID_TL, CENTER_LID_BR, CENTER_LID_TR};
  
  moveMultipleServos(channels, startAngles, endAngles, 6);
  
  Serial.println(F("CIRCULAR LOOK AROUND COMPLETE"));
  Serial.println(F("Eye returned to neutral position"));
}


// Lighter smoothing specifically for idle motion 
void moveMultipleServosIdle(int channels[], int startAngles[], int endAngles[], int numServos, bool isDramatic = false) {
  // Adjustable smoothing parameters based on movement type
  int smoothSteps, smoothDelay;
  float accelFactor;
  
  if (isDramatic) {
    // Randomized parameters 
    smoothSteps = random(DRAMATIC_SMOOTH_STEPS_MIN, DRAMATIC_SMOOTH_STEPS_MAX + 1);
    smoothDelay = random(DRAMATIC_SMOOTH_DELAY_MIN, DRAMATIC_SMOOTH_DELAY_MAX + 1);
    accelFactor = DRAMATIC_ACCEL_FACTOR;
    Serial.print(F("Dramatic smooth move ("));
    Serial.print(smoothSteps);
    Serial.print(F(" steps, "));
    Serial.print(smoothDelay);
    Serial.print(F("ms delay): "));
  } else {
    // Lighter smoothing for regural idle
    smoothSteps = 15;
    smoothDelay = 15;
    accelFactor = 2.0;
    Serial.print(F("Idle smooth move: "));
  }
  
  for (int i = 0; i < numServos; i++) {
    Serial.print(F("Ch"));
    Serial.print(channels[i]);
    Serial.print(F(":"));
    Serial.print(startAngles[i]);
    Serial.print(F("->"));
    Serial.print(endAngles[i]);
    if (i < numServos - 1) Serial.print(F(", "));
  }
  Serial.println();
  
  for (int step = 0; step <= smoothSteps; step++) {
    float t = (float)step / smoothSteps;
    
    // Apply exponential ease-in-out curve
    float smoothT;
    if (t < 0.5) {
      smoothT = pow(2 * t, accelFactor) / 2;
    } else {
      smoothT = 1 - pow(2 * (1 - t), accelFactor) / 2;
    }
    
    // Move all servos simultaneously
    for (int i = 0; i < numServos; i++) {
      int currentAngle = startAngles[i] + (int)((endAngles[i] - startAngles[i]) * smoothT);
      setServoCorrected(channels[i], currentAngle);
    }
    
    delay(smoothDelay);
  }
}


void idleMotion() {
  Serial.println(F("=== NATURAL IDLE MOTION ==="));
  Serial.println(F("Simulating natural eye behavior at rest"));
  Serial.println(F("Press any key to stop idle motion..."));
  
  // Clear any pending serial input
  while(Serial.available()) Serial.read();
  
  // Initialize idle state variables
  int currentEyeLR = CENTER_EYE_LR;
  int currentEyeUD = CENTER_EYE_UD;
  int currentLidBL = CENTER_LID_BL;
  int currentLidTL = CENTER_LID_TL;
  int currentLidBR = CENTER_LID_BR;
  int currentLidTR = CENTER_LID_TR;
  
  // Target positions for gradual drift
  int targetEyeLR = CENTER_EYE_LR;
  int targetEyeUD = CENTER_EYE_UD;
  
  unsigned long lastUpdate = millis();
  unsigned long lastBlink = millis();
  int idleCycles = 0;
  
  // Start from center
  setAllToCenter();
  delay(1000);
  
  Serial.println(F("Idle motion active - natural eye behavior with smooth drift..."));
  
  while (!Serial.available()) {
    unsigned long currentTime = millis();
    
    // Add some randomness to update interval (120-180ms)
    int updateInterval = IDLE_UPDATE_INTERVAL + random(-30, 30);
    
    if (currentTime - lastUpdate >= updateInterval) {
      lastUpdate = currentTime;
      idleCycles++;
      
      bool moved = false;
      
      // Microsaccades - tiny rapid eye movements
      if (random(100) < MICROSACCADE_CHANCE) {
        Serial.println(F("Microsaccade"));
        
        int microLR = currentEyeLR + random(-MICROSACCADE_SIZE, MICROSACCADE_SIZE + 1);
        int microUD = currentEyeUD + random(-MICROSACCADE_SIZE, MICROSACCADE_SIZE + 1);
        
        // Keep within idle bounds
        microLR = constrain(microLR, CENTER_EYE_LR - MAX_IDLE_OFFSET, CENTER_EYE_LR + MAX_IDLE_OFFSET);
        microUD = constrain(microUD, CENTER_EYE_UD - MAX_IDLE_OFFSET, CENTER_EYE_UD + MAX_IDLE_OFFSET);
        
        // Quick microsaccade movement
        setServoCorrected(SERVO_LOOK_LR, microLR);
        setServoCorrected(SERVO_LOOK_UD, microUD);
        
        currentEyeLR = microLR;
        currentEyeUD = microUD;
        moved = true;
      }
      // Dramatic eye movements - looking in specific directions with smooth motion
      if (random(100) < DRAMATIC_MOVE_CHANCE) {
        Serial.println(F("Dramatic eye movement"));
        
        // Store previous positions
        int prevEyeLR = currentEyeLR;
        int prevEyeUD = currentEyeUD;
        
        // Define dramatic movement types
        int moveType = random(8); // 8 different movement types
        int newEyeLR = currentEyeLR;
        int newEyeUD = currentEyeUD;
        
        switch(moveType) {
          case 0: // Look far left
            newEyeLR = CENTER_EYE_LR - random(20, DRAMATIC_MOVE_RANGE + 1);
            Serial.println(F("  -> Looking LEFT"));
            break;
          case 1: // Look far right
            newEyeLR = CENTER_EYE_LR + random(20, DRAMATIC_MOVE_RANGE + 1);
            Serial.println(F("  -> Looking RIGHT"));
            break;
          case 2: // Look up
            newEyeUD = CENTER_EYE_UD + random(15, DRAMATIC_MOVE_RANGE - 10);
            Serial.println(F("  -> Looking UP"));
            break;
          case 3: // Look down
            newEyeUD = CENTER_EYE_UD - random(15, DRAMATIC_MOVE_RANGE - 10);
            Serial.println(F("  -> Looking DOWN"));
            break;
          case 4: // Look left-up (LU)
            newEyeLR = CENTER_EYE_LR - random(15, 25);
            newEyeUD = CENTER_EYE_UD + random(10, 20);
            Serial.println(F("  -> Looking LEFT-UP"));
            break;
          case 5: // Look left-down (LD)
            newEyeLR = CENTER_EYE_LR - random(15, 25);
            newEyeUD = CENTER_EYE_UD - random(10, 20);
            Serial.println(F("  -> Looking LEFT-DOWN"));
            break;
          case 6: // Look right-up (RU)
            newEyeLR = CENTER_EYE_LR + random(15, 25);
            newEyeUD = CENTER_EYE_UD + random(10, 20);
            Serial.println(F("  -> Looking RIGHT-UP"));
            break;
          case 7: // Look right-down (RD)
            newEyeLR = CENTER_EYE_LR + random(15, 25);
            newEyeUD = CENTER_EYE_UD - random(10, 20);
            Serial.println(F("  -> Looking RIGHT-DOWN"));
            break;
        }
        
        // Constrain to safe ranges
        newEyeLR = constrain(newEyeLR, CENTER_EYE_LR - DRAMATIC_MOVE_RANGE, CENTER_EYE_LR + DRAMATIC_MOVE_RANGE);
        newEyeUD = constrain(newEyeUD, CENTER_EYE_UD - (DRAMATIC_MOVE_RANGE - 10), CENTER_EYE_UD + (DRAMATIC_MOVE_RANGE - 10));
        
        // dramatic smooth movement with randomized parameters
        int channels[2] = {SERVO_LOOK_LR, SERVO_LOOK_UD};
        int startAngles[2] = {prevEyeLR, prevEyeUD};
        int endAngles[2] = {newEyeLR, newEyeUD};
        
        moveMultipleServosIdle(channels, startAngles, endAngles, 2, true); 
        
        currentEyeLR = newEyeLR;
        currentEyeUD = newEyeUD;
        moved = true;
        
        
        delay(random(500, 1500)); // Hold dramatic for 0.5-1.5 seconds
        
       
        if (random(100) < 60) { // 60% chance to return to center
          Serial.println(F("  -> Partial return toward center"));
          
          int returnLR = currentEyeLR + (CENTER_EYE_LR - currentEyeLR) / 3;
          int returnUD = currentEyeUD + (CENTER_EYE_UD - currentEyeUD) / 3;
          
          int returnStartAngles[2] = {currentEyeLR, currentEyeUD};
          int returnEndAngles[2] = {returnLR, returnUD};
          
          moveMultipleServosIdle(channels, returnStartAngles, returnEndAngles, 2, false); // Regular smoothing for return
          
          currentEyeLR = returnLR;
          currentEyeUD = returnUD;
        }
      }
      // Slow drift - gradual position changes with SMOOTHING
      else if (random(100) < DRIFT_CHANCE) {
        Serial.println(F("Smooth position drift"));
        
   
        int prevEyeLR = currentEyeLR; // Store previous positions
        int prevEyeUD = currentEyeUD;
        
        // Set new drift targets
        targetEyeLR = CENTER_EYE_LR + random(-MAX_IDLE_OFFSET, MAX_IDLE_OFFSET + 1);
        targetEyeUD = CENTER_EYE_UD + random(-MAX_IDLE_OFFSET, MAX_IDLE_OFFSET + 1);
        
        // move partway toward targets
        int stepSizeLR = (targetEyeLR - currentEyeLR) / 3; // Move 1/3 of the way
        int stepSizeUD = (targetEyeUD - currentEyeUD) / 3;
        
        // minimum movement of 1 degree if there's a difference
        if (stepSizeLR == 0 && targetEyeLR != currentEyeLR) {
          stepSizeLR = (targetEyeLR > currentEyeLR) ? 1 : -1;
        }
        if (stepSizeUD == 0 && targetEyeUD != currentEyeUD) {
          stepSizeUD = (targetEyeUD > currentEyeUD) ? 1 : -1;
        }
        
        currentEyeLR += stepSizeLR;
        currentEyeUD += stepSizeUD;
        
        // smooth movement for drift if there's significant movement
        if (abs(currentEyeLR - prevEyeLR) > 1 || abs(currentEyeUD - prevEyeUD) > 1) {
          
          int channels[2] = {SERVO_LOOK_LR, SERVO_LOOK_UD}; //coordinated smooth movement for both axes
          int startAngles[2] = {prevEyeLR, prevEyeUD};
          int endAngles[2] = {currentEyeLR, currentEyeUD};
          
          // lighter smoothing for idle 
          moveMultipleServosIdle(channels, startAngles, endAngles, 2, false);
        } else {
          // For tiny movements, direct positioning
          setServoCorrected(SERVO_LOOK_LR, currentEyeLR);
          setServoCorrected(SERVO_LOOK_UD, currentEyeUD);
        }
        
        moved = true;
      }
      
      // Subtle eyelid adjustments
      if (random(100) < 20) { // 20% chance of lid adjustment
        int lidAdjust = random(-2, 3); 
        
        // Randomly adjust one or more eyelids
        if (random(100) < 50) {
          currentLidBL = constrain(CENTER_LID_BL + lidAdjust, CENTER_LID_BL - 5, CENTER_LID_BL + 5);
          setServoCorrected(SERVO_LID_BL, currentLidBL);
        }
        if (random(100) < 50) {
          currentLidTL = constrain(CENTER_LID_TL + lidAdjust, CENTER_LID_TL - 5, CENTER_LID_TL + 5);
          setServoCorrected(SERVO_LID_TL, currentLidTL);
        }
        if (random(100) < 50) {
          currentLidBR = constrain(CENTER_LID_BR + lidAdjust, CENTER_LID_BR - 5, CENTER_LID_BR + 5);
          setServoCorrected(SERVO_LID_BR, currentLidBR);
        }
        if (random(100) < 50) {
          currentLidTR = constrain(CENTER_LID_TR + lidAdjust, CENTER_LID_TR - 5, CENTER_LID_TR + 5);
          setServoCorrected(SERVO_LID_TR, currentLidTR);
        }
      }
      
      // Natural blinking
      if (currentTime - lastBlink >= max(1500, 2000 + random(1000, 4000))) { // fewer chance of double blinking )3-6 sec for casual blink)
        Serial.println(F("Natural blink"));
        performNaturalBlink();
        lastBlink = currentTime;
        
        // Reset eyelid positions after blink
        currentLidBL = CENTER_LID_BL;
        currentLidTL = CENTER_LID_TL;
        currentLidBR = CENTER_LID_BR;
        currentLidTR = CENTER_LID_TR;
      }
      
      // Occasional return toward center
      if (idleCycles % 20 == 0) { // Every 20 cycles
        if (abs(currentEyeLR - CENTER_EYE_LR) > MAX_IDLE_OFFSET/2 || 
            abs(currentEyeUD - CENTER_EYE_UD) > MAX_IDLE_OFFSET/2) {
          Serial.println(F("Gentle smooth return to center"));
          
          // Store current position
          int prevEyeLR = currentEyeLR;
          int prevEyeUD = currentEyeUD;
          
          // Gradually return toward center
          if (currentEyeLR > CENTER_EYE_LR) currentEyeLR--;
          else if (currentEyeLR < CENTER_EYE_LR) currentEyeLR++;
          
          if (currentEyeUD > CENTER_EYE_UD) currentEyeUD--;
          else if (currentEyeUD < CENTER_EYE_UD) currentEyeUD++;
          
          // Use smooth movement for return to center
          int channels[2] = {SERVO_LOOK_LR, SERVO_LOOK_UD};
          int startAngles[2] = {prevEyeLR, prevEyeUD};
          int endAngles[2] = {currentEyeLR, currentEyeUD};
          
          moveMultipleServosIdle(channels, startAngles, endAngles, 2, false);
        }
      }
      
      // Status update every 50 cycles
      if (idleCycles % 50 == 0) {
        Serial.print(F("Idle cycles: "));
        Serial.print(idleCycles);
        Serial.print(F(" | Position: LR="));
        Serial.print(currentEyeLR);
        Serial.print(F(" UD="));
        Serial.println(currentEyeUD);
      }
    }
    
    delay(10); // Small delay to prevent overwhelming the system
  }
  
  // Clear the key press that stopped idle motion
  while(Serial.available()) Serial.read();
  
  Serial.println(F("Idle motion stopped. Returning to center..."));
  setAllToCenter();
  Serial.println(F("=== IDLE MOTION COMPLETE ==="));
}

void performNaturalBlink() {
  // Natural blink sequence 
  int blinkAmount = 35; 
  
  // Slightly staggered blink timing for realism
  setServoCorrected(SERVO_LID_BL, CENTER_LID_BL + blinkAmount);
  setServoCorrected(SERVO_LID_BR, CENTER_LID_BR + blinkAmount);
  delay(50);
  
  setServoCorrected(SERVO_LID_TL, CENTER_LID_TL + blinkAmount - 2); // Top lids slightly less
  setServoCorrected(SERVO_LID_TR, CENTER_LID_TR + blinkAmount - 2);
  delay(BLINK_DURATION - 100);
  
  // Open eyes with slight stagger
  setServoCorrected(SERVO_LID_TL, CENTER_LID_TL);
  setServoCorrected(SERVO_LID_TR, CENTER_LID_TR);
  delay(30);
  
  setServoCorrected(SERVO_LID_BL, CENTER_LID_BL);
  setServoCorrected(SERVO_LID_BR, CENTER_LID_BR);
}
