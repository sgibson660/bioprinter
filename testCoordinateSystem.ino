#include <TimerOne.h>
// ------------------- X, Y, Z (independent counters, same possible speed) -------------------
volatile uint16_t stepCompareX = 0;
volatile uint16_t stepCounterX = 0;
volatile bool directionX = false;
volatile long positionX = 0;  // current position or steps taken

volatile uint16_t stepCompareY = 0;
volatile uint16_t stepCounterY = 0;
volatile bool directionY = false;
volatile long positionY = 0;

volatile uint16_t stepCompareZ = 0;
volatile uint16_t stepCounterZ = 0;
volatile bool directionZ = false;
volatile long positionZ = 0;

// ------------------- Syringes: S1, S2, S3 -------------------
volatile uint16_t stepCompareS1 = 0;
volatile uint16_t stepCounterS1 = 0;
volatile bool directionS1 = false;
volatile long positionS1 = 0;

volatile uint16_t stepCompareS2 = 0;
volatile uint16_t stepCounterS2 = 0;
volatile bool directionS2 = false;
volatile long positionS2 = 0;

volatile uint16_t stepCompareS3 = 0;
volatile uint16_t stepCounterS3 = 0;
volatile bool directionS3 = false;
volatile long positionS3 = 0;

// Serial buffer
const int buffer_size = 102;
uint8_t data[buffer_size];

// ------------------- Define Pins and Motor Characteristics -------------------
#define ENAs3 41  //define Enable pin syringe #1 motor
#define DIRs3 40  //define Direction pin syringe #1 motor
#define PULs3 39  //define Pulse pin syringe #1 motor

#define ENAs2 38  //define Enable pin syringe #2 motor
#define DIRs2 37  //define Direction pin syringe #2 motor
#define PULs2 36  //define Pulse pin syringe #2 motor

#define ENAs1 35  //define Enable pin syringe #3 motor
#define DIRs1 34  //define Direction pin syringe #3 motor
#define PULs1 33  //define Pulse pin syringe #3 motor

#define ENAx 44  //define Enable pin x motor
#define DIRx 43  //define Direction pin x motor
#define PULx 42  //define Pulse pin x motor

#define ENAly 47  //define Enable pin left y motor
#define DIRly 46  //define Direction pin left y motor
#define PULly 45  //define Pulse pin left y motor

#define ENAry 50  //define Enable pin right y motor
#define DIRry 49  //define Direction pin right y motor
#define PULry 48  //define Pulse pin right y motor

#define ENAz 53  //define Enable pin z motor
#define DIRz 52  //define Direction pin z motor
#define PULz 51  //define Pulse pin z motor

// Microswitches for homing (normally open, closed when pressed)
#define MSW_X 2  // Microswitch for X axis
#define MSW_Y 3  // Microswitch for Y axis
#define MSW_Z 4  // Microswitch for Z axis
#define MSW_S1 6 // Microswitch for Syringe 1
#define MSW_S2 9 // Microswitch for Syringe 2
#define MSW_S3 8 // Microswitch for Syring 3

// Define the microswitch pin for emergency stop
#define EMERGENCY_STOP_PIN 5  //Microswitch for emergency stop

// #define MicroStepping 200
#define FullRotation 1600
#define FullRotationSyringe 3200

#define TravelPerRotation 10  // mm per revolution
#define TravelPerRotationSyringe 0.1 //mm per revolution

// need to re-measure these
#define Max_Distance_X 120    //mm
#define Max_Distance_Y 126.8  //mm
#define Max_Distance_Z 87.45  //mm
#define Max_Distance_S 51     //mm measured 52.1

#define MAX_STEPS_X round((Max_Distance_X / TravelPerRotation) * FullRotation)  // Set this based on screw length for X axis
#define MAX_STEPS_Y round((Max_Distance_Y / TravelPerRotation) * FullRotation)  // Set this based on screw length for Y axis
#define MAX_STEPS_Z round((Max_Distance_Z / TravelPerRotation) * FullRotation)  // Set this based on screw length for Z axis
#define MAX_STEPS_S round((Max_Distance_S / TravelPerRotationSyringe) * FullRotationSyringe)  // Set this based on screw length for syringe 1

// Define motor IDs
#define MOTOR_S3 5
#define MOTOR_S2 4
#define MOTOR_S1 3
#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2

long currentStepsX = MAX_STEPS_X - 50;  // To track the current position of X axis
long currentStepslY = MAX_STEPS_Y - 50;
long currentStepsrY = MAX_STEPS_Y - 50;  // To track the current position of Y axis
long currentStepsZ = MAX_STEPS_Z - 50;   // To track the current position of Z axis
long currentStepsS1 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 1
long currentStepsS2 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 2
long currentStepsS3 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 3

int speed;
int xTarget;
int yTarget;
int zTarget;
int s1Target;

// ----------------------- Pin and Arduino Setup --------------------------
void setup() {
  Serial.begin(9600);

  // Setup microswitches
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(MSW_X, INPUT_PULLUP);
  pinMode(MSW_Y, INPUT_PULLUP);
  pinMode(MSW_Z, INPUT_PULLUP);
  // Z, Y, X Motors
  pinMode(ENAz, OUTPUT);
  pinMode(DIRz, OUTPUT);
  pinMode(PULz, OUTPUT);
  pinMode(ENAly, OUTPUT);
  pinMode(DIRly, OUTPUT);
  pinMode(PULly, OUTPUT);
  pinMode(ENAry, OUTPUT);
  pinMode(DIRry, OUTPUT);
  pinMode(PULry, OUTPUT);
  pinMode(ENAx, OUTPUT);
  pinMode(DIRx, OUTPUT);
  pinMode(PULx, OUTPUT);
  // Syringes 1,2,3
  pinMode(ENAs1, OUTPUT);
  pinMode(DIRs1, OUTPUT);
  pinMode(PULs1, OUTPUT);
  pinMode(ENAs2, OUTPUT);
  pinMode(DIRs2, OUTPUT);
  pinMode(PULs2, OUTPUT);
  pinMode(ENAs3, OUTPUT);
  pinMode(DIRs3, OUTPUT);
  pinMode(PULs3, OUTPUT);

  // Enable the motors to move
  digitalWrite(ENAz, HIGH);
  digitalWrite(ENAly, HIGH);
  digitalWrite(ENAry, HIGH);
  digitalWrite(ENAx, HIGH);
  digitalWrite(ENAs1, HIGH);
  digitalWrite(ENAs2, HIGH);
  digitalWrite(ENAs3, HIGH);

  // Initialize Timer1 at 1 kHz (1000 us)
  Timer1.initialize(100);
  Timer1.attachInterrupt(stepISR);

  homeAxis();  // your existing homing routine
}

// ----------------- Main Function Loop ----------------------------
void loop() {
  // Check emergency stop
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    // Timer1.stop();
    Serial.println("Stop");

    // Move the Z axis up out of the print
    const int stepsToMove = 1500;
    for (int i = 0; i < stepsToMove; i++) {
      stepMotor(DIRz, PULz, LOW, 200, &currentStepsZ, MAX_STEPS_Z);
    }

    // Stop all motors
    stepCompareX = 0;
    stepCompareY = 0;
    stepCompareZ = 0;
    stepCompareS1 = 0;
    stepCompareS2 = 0;
    stepCompareS3 = 0;

    currentStepsX = MAX_STEPS_X - 50;  // To track the current position of X axis
    currentStepslY = MAX_STEPS_Y - 50;
    currentStepsrY = MAX_STEPS_Y - 50;  // To track the current position of Y axis
    currentStepsZ = MAX_STEPS_Z - 50;   // To track the current position of Z axis
    currentStepsS1 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 1
    currentStepsS2 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 2
    currentStepsS3 = MAX_STEPS_S - 0.001;  // To track the current position of syringe 3

    // Flush the data sent from MATLAB
    Serial.flush();
    homeAxis();

    currentStepsX = 0;  // To track the current position of X axis
    currentStepslY = 0;
    currentStepsrY = 0;  // To track the current position of Y axis
    currentStepsZ = 0;   // To track the current position of Z axis
    currentStepsS1 = 0;  // To track the current position of syringe 1
    currentStepsS2 = 0;  // To track the current position of syringe 2
    currentStepsS3 = 0;  // To track the current position of syringe 3
    Serial.flush();
  }

  // Read from Serial
  if (Serial.available() > 0) {
    // Read incoming string
    String data = Serial.readString();
    int spaceIndexes[4];                  // Array to store space indexes
    int index = -1;

    for (int i = 0; i < 4; i++) {
      index = data.indexOf(' ', index + 1);
      if (index == -1) break;  // Stop if no more spaces are found
      spaceIndexes[i] = index;
    }

    // Extract values using spaceIndexes
    int speedProfile = data.substring(0, spaceIndexes[0]).toInt();
    if (speedProfile == 1) {
      speed = 15;
    } else if (speedProfile == 0) {
      speed = 14;
    }
    xTarget = data.substring(spaceIndexes[0] + 1, spaceIndexes[1]).toFloat();
    yTarget = data.substring(spaceIndexes[1] + 1, spaceIndexes[2]).toFloat();
    zTarget = data.substring(spaceIndexes[2] + 1, spaceIndexes[3]).toFloat();
    s1Target = data.substring(spaceIndexes[3] + 1, data.length()-1).toFloat();
    //float s2Target = data.substring(spaceIndexes[4] + 1, spaceIndexes[5]).toFloat();
    //float s3Target = data.substring(spaceIndexes[5] + 1).toFloat();  // Last value

    long xTargetSteps = round((xTarget / TravelPerRotation) * FullRotation);
    long xSteps = xTargetSteps - currentStepsX;
    int xDir = (xTarget >= abs((currentStepsX * TravelPerRotation) / FullRotation)) ? 0 : 1;
    xSteps = abs(xSteps);
    
    long yTargetSteps = round((yTarget / TravelPerRotation) * FullRotation);
    long ySteps = yTargetSteps - currentStepsrY;
    int yDir = (yTarget >= abs((currentStepsrY * TravelPerRotation) / FullRotation))? 0 : 1;
    ySteps = abs(ySteps);

    long zTargetSteps = round((zTarget / TravelPerRotation) * FullRotation);
    long zSteps = zTargetSteps - currentStepsZ;
    int zDir = (zTarget >= abs((currentStepsZ * TravelPerRotation) / FullRotation)) ? 0 : 1;
    zSteps = abs(zSteps);

    long s1TargetSteps = round((s1Target / TravelPerRotationSyringe) * FullRotationSyringe);
    long s1Steps = s1TargetSteps - currentStepsS1;
    int s1Dir = 0; 
    s1Steps = abs(s1Steps/20);

    setMotorSpeedAndDir(0, speed, xDir, xSteps);    // X
    setMotorSpeedAndDir(1, speed, yDir, ySteps);    // Y
    setMotorSpeedAndDir(2, speed, zDir, zSteps);    // Z
    setMotorSpeedAndDir(3, speed, s1Dir, s1Steps);  // S1
    // setMotorSpeedAndDir(4, speedProfile, s2Dir,s2Steps); // S2
    // setMotorSpeedAndDir(5, speedProfile, s3Dir,s3Steps); // S3

    // Wait for motors to complete movement
    while (motorsAreMoving()) {
      delay(1);  // Small delay to avoid CPU overload
      // Serial.println("M");
    }

    Serial.println("D");
  }
}

// ---------------------- Helper Functions -------------------------
bool motorsAreMoving() {
  return stepCounterX > 0 || stepCounterY > 0 || stepCounterZ > 0 || stepCounterS1 > 0;
}

void stepISR() {
  // ---------------- X motor ----------------
  if (stepCompareX > 0 && stepCounterX > 0) {
    stepCounterX--;  // Count down steps
    if ((stepCounterX % stepCompareX) == 0) {
      if (digitalRead(MSW_X) == LOW) {
        Serial.println("X Maximum");
      } else {
        digitalWrite(PULx, !digitalRead(PULx));
        if (digitalRead(PULx) == HIGH) {  // Only update on rising edge
          currentStepsX += (directionX ? 1 : -1);
          positionX = currentStepsX; 
        }
      }
    }
  }

  // ---------------- Y motor ----------------
  if (stepCompareY > 0 && stepCounterY > 0) {
    stepCounterY--;
    if ((stepCounterY % stepCompareY) == 0) {
      if (digitalRead(MSW_Y) == LOW) {
        Serial.println("Y Maximum");
      } else {
        digitalWrite(PULly, !digitalRead(PULly));
        digitalWrite(PULry, !digitalRead(PULry));
        if (digitalRead(PULly) == HIGH) {
          currentStepslY += (directionY ? 1 : -1);
          currentStepsrY += (directionY ? 1 : -1);
          positionY = currentStepslY; 
        }
      }
    }
  }

  // ---------------- Z motor ----------------
  if (stepCompareZ > 0 && stepCounterZ > 0) {
    stepCounterZ--;
    if ((stepCounterZ % stepCompareZ) == 0) {
      if (digitalRead(MSW_Z) == LOW) {
        Serial.println("Z Maximum");
      } else {
        digitalWrite(PULz, !digitalRead(PULz));
        if (digitalRead(PULz) == HIGH) {
          currentStepsZ += (directionZ ? 1 : -1);
          positionZ = currentStepsZ; 
        }
      }
    }
  }

  // ---------------- Syringe 1 ----------------
  if (stepCompareS1 > 0 && stepCounterS1 > 0) {
    stepCounterS1--;
    if ((stepCounterS1 % stepCompareS1) == 0) {
      digitalWrite(PULs1, !digitalRead(PULs1));
      if (digitalRead(PULs1) == HIGH) {
        currentStepsS1 += (directionS1 ? 1 : -1);
        positionS1 = currentStepsS1; 
      }
    }
  }

  // ---------------- Syringe 2 ----------------
  if (stepCompareS2 > 0 && stepCounterS2 > 0) {
    stepCounterS2--;
    if ((stepCounterS2 % stepCompareS2) == 0) {
      digitalWrite(PULs2, !digitalRead(PULs2));
      if (digitalRead(PULs2) == HIGH) {
        currentStepsS2 += (directionS2 ? 1 : -1);
        positionS2 = currentStepsS2; 
      }
    }
  }

  // ---------------- Syringe 3 ----------------
  if (stepCompareS3 > 0 && stepCounterS3 > 0) {
    stepCounterS3--;
    if ((stepCounterS3 % stepCompareS3) == 0) {
      digitalWrite(PULs3, !digitalRead(PULs3));
      if (digitalRead(PULs3) == HIGH) {
        currentStepsS3 += (directionS3 ? 1 : -1);
        positionS3 = currentStepsS3; 
      }
    }
  }

  // **Update target positions once motion is complete**
  if (stepCounterX == 0) positionX = currentStepsX;
  if (stepCounterY == 0) positionY = currentStepsrY;
  if (stepCounterZ == 0) positionZ = currentStepsZ;
  if (stepCounterS1 == 0) positionS1 = currentStepsS1;
  if (stepCounterS2 == 0) positionS2 = currentStepsS2;
  if (stepCounterS3 == 0) positionS3 = currentStepsS3;
}

void setMotorSpeedAndDir(byte motorId, byte speed, byte dir, int steps) {
  uint16_t compareVal = 0;
  if (speed > 0) {
    // Example mapping: speed=1 => ~500 => slow stepping,
    //                  speed=15 => 10 => fastest stepping
    compareVal = map(speed, 1, 15, 500, 10);
  }

  switch (motorId) {
    case MOTOR_X:
      stepCompareX = compareVal;
      stepCounterX = abs(steps)+compareVal;
      directionX = dir;
      digitalWrite(DIRx, dir);  // physically set direction pin
      break;

    case MOTOR_Y:
      stepCompareY = compareVal;
      stepCounterY = abs(steps)+compareVal;
      directionY = dir;
      digitalWrite(DIRly, dir);
      digitalWrite(DIRry, dir);
      break;

    case MOTOR_Z:
      stepCompareZ = compareVal;
      stepCounterZ = abs(steps)+compareVal;
      directionZ = dir;
      digitalWrite(DIRz, dir);
      break;

    case MOTOR_S1:
      stepCompareS1 = compareVal;
      stepCounterS1 = abs(steps)+compareVal;
      directionS1 = dir;
      digitalWrite(DIRs1, dir);
      break;

    case MOTOR_S2:
      stepCompareS2 = compareVal;
      stepCounterS2 = abs(steps)+compareVal;
      directionS2 = dir;
      digitalWrite(DIRs2, dir);
      break;

    case MOTOR_S3:
      stepCompareS3 = compareVal;
      stepCounterS3 = abs(steps)+compareVal;
      directionS3 = dir;
      digitalWrite(DIRs3, dir);
      break;
  }
}

void homeAxis() {
  Timer1.stop();
  while (digitalRead(MSW_Y) == HIGH) {
    stepMotor(DIRly, PULly, HIGH, 200, &currentStepslY, MAX_STEPS_Y);
    stepMotor(DIRry, PULry, HIGH, 200, &currentStepsrY, MAX_STEPS_Y);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRly, PULly, LOW, 200, &currentStepslY, MAX_STEPS_Y);
    stepMotor(DIRry, PULry, LOW, 200, &currentStepsrY, MAX_STEPS_Y);
  }
  while (digitalRead(MSW_Y) == HIGH) {
    stepMotor(DIRly, PULly, HIGH, 1000, &currentStepslY, MAX_STEPS_Y);
    stepMotor(DIRry, PULry, HIGH, 1000, &currentStepsrY, MAX_STEPS_Y);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRly, PULly, LOW, 1000, &currentStepslY, MAX_STEPS_Y);
    stepMotor(DIRry, PULry, LOW, 1000, &currentStepsrY, MAX_STEPS_Y);
  }
  currentStepslY = 0;  // Reset Y-axis step count after homing
  currentStepsrY = 0;
  Serial.println("Y axis homed");

  while (digitalRead(MSW_X) == HIGH) {
    stepMotor(DIRx, PULx, HIGH, 200, &currentStepsX, MAX_STEPS_X);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRx, PULx, LOW, 200, &currentStepsX, MAX_STEPS_X);
  }
  while (digitalRead(MSW_X) == HIGH) {
    stepMotor(DIRx, PULx, HIGH, 1000, &currentStepsX, MAX_STEPS_X);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRx, PULx, LOW, 1000, &currentStepsX, MAX_STEPS_X);
  }
  currentStepsX = 0;  // Reset X-axis step count after homing
  Serial.println("X axis homed");

  while (digitalRead(MSW_Z) == HIGH) {
    stepMotor(DIRz, PULz, HIGH, 200, &currentStepsZ, MAX_STEPS_Z);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRz, PULz, LOW, 200, &currentStepsZ, MAX_STEPS_Z);
  }
  while (digitalRead(MSW_Z) == HIGH) {
    stepMotor(DIRz, PULz, HIGH, 1000, &currentStepsZ, MAX_STEPS_Z);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRz, PULz, LOW, 1000, &currentStepsZ, MAX_STEPS_Z);
  }
  currentStepsZ = 0;  // Reset Z-axis step count after homing
  Serial.println("Z axis homed");

  while (digitalRead(MSW_S1) == HIGH) {
    stepMotor(DIRs1, PULs1, HIGH, 100, &currentStepsS1, MAX_STEPS_S);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRs1, PULs1, LOW, 100, &currentStepsS1, MAX_STEPS_S);
  }
  while (digitalRead(MSW_S1) == HIGH) {
    stepMotor(DIRs1, PULs1, HIGH, 500, &currentStepsS1, MAX_STEPS_S);
  }
  for (int i = 0; i <= 255; i++) {
    stepMotor(DIRs1, PULs1, LOW, 500, &currentStepsS1, MAX_STEPS_S);
  }
  currentStepsS1 = 0;  // Reset X-syringe step count after homing
  Serial.println("Syringe 1 homed");

  while(digitalRead(MSW_S2) == HIGH){
    stepMotor(DIRs2, PULs2, HIGH, 100, &currentStepsS2, MAX_STEPS_S);
  }
  for (int i = 0; i <= 255; i++){
    stepMotor(DIRs2, PULs2, LOW, 100, &currentStepsS2, MAX_STEPS_S);
  }
  currentStepsS2 = 0;  // Reset X-axis step count after homing
  Serial.println("Syringe 2 homed");

  while(digitalRead(MSW_S3) == HIGH){
    stepMotor(DIRs3, PULs3, HIGH, 100, &currentStepsS3, MAX_STEPS_S);
  }
  for (int i = 0; i <= 255; i++){
    stepMotor(DIRs3, PULs3, LOW, 100, &currentStepsS3, MAX_STEPS_S);
  }
  currentStepsS3 = 0;  // Reset X-axis step count after homing
  Serial.println("Syringe 3 homed");
  Timer1.start();
}

void stepMotor(int DIR, int PUL, bool direction, int delayTime, long *currentSteps, long maxSteps) {
  long tempSteps = *currentSteps;
  // Check if the movement is within the limit

  if (tempSteps <= maxSteps && tempSteps >= 0) {
    digitalWrite(DIR, direction);  // Set direction
    digitalWrite(PUL, HIGH);       // Pulse high
    delayMicroseconds(delayTime);  // Delay
    digitalWrite(PUL, LOW);        // Pulse low
    delayMicroseconds(delayTime);  // Delay

    // Update the current step count
    if (direction == LOW) {
      if (DIR == DIRz) {
        tempSteps++;
      } else {
        tempSteps++;
      }
    } else {
      if (DIR == DIRz) {
        tempSteps--;
      } else {
        tempSteps--;
      }
    }
    // Print current step count for debugging
    *currentSteps = tempSteps;
  }
}
