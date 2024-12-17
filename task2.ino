#include <LiquidCrystal.h> // Include LiquidCrystal library for the LCD
#include <MPU6050.h>  // Include MPU6050 library

// Motor driver pins
int motorPin1 = 4; // IN1 pin on motor driver connected to pin 4 in arduino
int motorPin2 = 5; // IN2 pin on motor driver connected to pin 5 in arduino
int enablePin = 9; // Enable pin on motor driver connected to pin 9 in arduino (the enable pin can be connected to ∿ pins in arduino)

// Encoder pins
int encoderPinA = 2; // encoder pin A is C1 pin on the DC motor, it is connected with pin 2 in arduino
int encoderPinB = 3; // encoder pin B is C2 pin on the DC motor, it is connected with pin 3 in arduino

// LCD initialization for parallel mode
LiquidCrystal lcd(7, 8, 10, 11, 12, 13);  // {LCD pin -> arduino pin} ==> {RS -> pin 7, E -> pin 8, D4 -> pin 10, D5 -> pin 11, D6 -> pin 12, D7 -> pin 13}

// MPU6050 initialization
MPU6050 mpu;
int16_t ax, ay, az;  // Accelerometer readings
float targetAngle = 0.0; // the target angle is initialized to 0

// Constants for encoder
const int TICKS_PER_REV = 300;  // CH-N20-3 encoder has PPR value equals 300
volatile int encoderPos = 0;    // Encoder position is initialized to 0
int lastEncoded = 0;            // To track the previous state

void setup() {
  // Motor pins setup, it sends data to motor driver, hence OUTPUT
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Encoder pins setup, it reads from the encoder, hence INPUT
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Attach interrupts for both encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Serial and MPU6050 setup
  Serial.begin(9600);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);  // Stop if MPU6050 connection fails
  }

  // LCD setup
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read MPU6050 accelerometer data
  mpu.getAcceleration(&ax, &ay, &az);

  // Map MPU6050 accelerometer X-axis data to -90 to 90 degrees
  targetAngle = map(ax, -17000, 17000, -90, 90);

  // Calculate current angle based on encoder position
  float angle = (encoderPos % TICKS_PER_REV) * 360.0 / TICKS_PER_REV;

  // Calculate error
  float error = targetAngle - angle; // The value of error is the required angle that should the motor rotate by
  int ierror= int(error); 
  // Control motor to reduce error
  if (abs(ierror) > 5) {
    controlMotor(error);
  } else {
    stopMotor();
  }

  // Print values to Serial Monitor
  Serial.print("Target Angle: ");
  Serial.print(targetAngle);
  Serial.print(" | Current Angle: ");
  Serial.print(angle);
  Serial.print(" | Error: ");
  Serial.println(error);

  // Print values to LCD
  lcd.setCursor(0, 0); // Line 1 on the LCD screen
  lcd.print("Target: ");
  lcd.print(targetAngle, 1);
  lcd.print("   "); // Clear extra digits

  lcd.setCursor(0, 1); // Line 2 on the LCD screen
  lcd.print("Current: ");
  lcd.print(angle, 1);
  lcd.print("   "); 

  delay(100);
}

// Function to control motor based on error
void controlMotor(float error) {
  int motorSpeed = constrain(abs(error) * 2, 0, 255);  // Scale speed based on error

  // The motor rotate towards the high value
  if (error > 5) {  // Rotate forward
    digitalWrite(motorPin1, HIGH); // IN1 high
    digitalWrite(motorPin2, LOW); // IN2 low
    analogWrite(enablePin, motorSpeed);
  } else if (error < -5) {  // Rotate backward
    digitalWrite(motorPin1, LOW); // IN1 low
    digitalWrite(motorPin2, HIGH); // IN2 high
    analogWrite(enablePin, motorSpeed);
  } else {
    stopMotor();
  }
}

// Stop the motor, set both terminals of the motor to have 0 volts then no rotation will occur
void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  analogWrite(enablePin, 0);
}

// Encoder ISR to update position
void updateEncoder() {
  int currentState = (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);

  // Determine direction based on state transition
  if ((lastEncoded == 0 && currentState == 1) || 
      (lastEncoded == 1 && currentState == 3) || 
      (lastEncoded == 3 && currentState == 2) || 
      (lastEncoded == 2 && currentState == 0)) {
    encoderPos++;  // Clockwise rotation
  } else if ((lastEncoded == 0 && currentState == 2) || 
             (lastEncoded == 2 && currentState == 3) || 
             (lastEncoded == 3 && currentState == 1) || 
             (lastEncoded == 1 && currentState == 0)) {
    encoderPos--;  // Counterclockwise rotation
  }

  lastEncoded = currentState; // update the value of lastEncoded to have the current state of the motor

}