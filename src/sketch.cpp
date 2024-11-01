#include <Arduino.h>
// #include <string.h>

// mcu input pins for joystick
const int joystick_Vert_Pin = 36;
const int joystick_Horz_Pin = 39;

// mcu output pins to motor driver input
const int motor1_In1 = 19;
const int motor1_In2 = 18;
const int motor1_PWM = 5;
const int motor2_In1 = 17;
const int motor2_In2 = 16;
const int motor2_PWM = 4;

// mcu input pins for motor's encoder
const int encoder1_C = 2;
const int encoder1_D = 15;
const int encoder2_C = 8;
const int encoder2_D = 7;

// Variables for speed, position, and heading calculations
int encoder1_Count = 0;
int encoder2_Count = 0;
int encoder1_Count_b = 0;
int encoder2_Count_b = 0;

unsigned long prevTime = 0;      // To calculate elapsed time
const int encoder_slots = 20;     // Number of slots in encoder disk
const float wheelDiameter = 0.020;      // Wheel diameter in meter
const float wheelBase = 0.15;           // Distance between wheels in meter


float linearVelocity = 0;
float angularVelocity = 0;
float linearVelocity_b = 0;
float angularVelocity_b = 0;

float posX = 0;  // robot's position X in meter
float posY = 0;
float heading = 0;  // robot's Heading angle in degree

// Interrupt service routines for encoder counting
void IRAM_ATTR encoder1_ISR() {
  if (digitalRead(encoder1_D))
    encoder1_Count++;
  else
    encoder1_Count--;
}

void IRAM_ATTR encoder2_ISR() {
  if (digitalRead(encoder2_D))
    encoder2_Count++;
  else
    encoder2_Count--;
}

// Function to set motor direction and speed
void setMotorSpeed(int motor, int direction, int speed) {
  float simEncode = 10 * speed / 255;
  float simSpeed = speed == 255 ? 255 : speed * 0.1;

  if (motor == 1) { // Motor 1
    if (speed == 0) {
      digitalWrite(motor1_In1, LOW);
      digitalWrite(motor1_In2, LOW);
    } else {
      digitalWrite(motor1_In1, direction == 1 ? HIGH : LOW);
      digitalWrite(motor1_In2, direction == 1 ? LOW : HIGH);
    }

    // 20241101: using ledc* API needs other dependences in Vscode-Wokwi-PlatformIO environment
    // https://github.com/espressif/arduino-esp32/issues/9169#issuecomment-2013294488
    // ledcWrite(motor1_PWM, speed);
    analogWrite(motor1_PWM, simSpeed);

    if (direction)
      encoder1_Count += simEncode;
    else
      encoder1_Count -= simEncode;
  } else if (motor == 2) { // Motor 2
    if (speed == 0) {
      digitalWrite(motor2_In1, LOW);
      digitalWrite(motor2_In2, LOW);
    } else {
      digitalWrite(motor2_In1, direction == 1 ? HIGH : LOW);
      digitalWrite(motor2_In2, direction == 1 ? LOW : HIGH);
    }
    // ledcWrite(motor2_PWM, speed);
    analogWrite(motor2_PWM, simSpeed);

    if (direction)
      encoder2_Count += simEncode;
    else
      encoder2_Count -= simEncode;
  }
}

// Function to calculate speed, position, and heading
void calculateOdometry() {
  // Calculate time elapsed
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // seconds
  prevTime = currentTime;

  int dt1 = encoder1_Count - encoder1_Count_b;
  int dt2 = encoder2_Count - encoder2_Count_b;
  encoder1_Count_b = encoder1_Count;
  encoder2_Count_b = encoder2_Count;

  // Calculate wheel speeds (m/s)
  static float distPerCount = (PI * wheelDiameter) / encoder_slots * 0.1;  // 0.5 gear rate
  float motorSpeed1 = (dt1 * distPerCount * 10) / deltaTime;  // multiply 10 for quick demonstrating
  float motorSpeed2 = (dt2 * distPerCount * 10) / deltaTime;

  // Calculate linear and angular velocity
  linearVelocity = (motorSpeed1 + motorSpeed2) / 2;
  angularVelocity = (motorSpeed1 - motorSpeed2) / wheelBase * 180/PI;  // heading angle, right is positive

  if (linearVelocity_b == linearVelocity && angularVelocity_b == angularVelocity) {
    return;
  }

  linearVelocity_b = linearVelocity;
  angularVelocity_b = angularVelocity;

  // Update robot's position and heading
  heading += angularVelocity * deltaTime;
  posX += linearVelocity * sin(heading) * deltaTime;
  posY += linearVelocity * cos(heading) * deltaTime;

  // Print speed, position, and heading for debugging
  char str[1024];
  memset(str, 0, sizeof str);
  // sprintf(str, "Speed (m/s): Motor1 = %d, Motor2 = %d", motorSpeed1, motorSpeed2);
  // Serial.println(str);

  Serial.print(deltaTime);  // also for print error (if do not print a num here, the following num will be nan..)
  Serial.print("--");
  sprintf(str, "Speed: tran=%.3f, rot=%.3f; Pos: x, y, r=%.3f, %.3f, %.3f", linearVelocity, angularVelocity, posX, posY, heading);
  Serial.println(str);
}

void speedCtrl() {
  // int vert = (analogRead(Joystick_Vert_Pin));
  // int horz = (analogRead(Joystick_Horz_Pin));
  int vert = map(analogRead(joystick_Vert_Pin), 0, 4095, -100, 100);
  int horz = map(analogRead(joystick_Horz_Pin), 0, 4095, -100, 100);
  // Serial.print(vert);
  // Serial.print(" -- ");
  // Serial.println(horz);

  // Serial.println("V=%d, H=%d", vert, horz);
  if (horz == 0) {
    if (vert > 0) {
      setMotorSpeed(1, 1, 255);
      setMotorSpeed(2, 1, 255);
    } else if (vert == 0) {
      setMotorSpeed(1, 1, 0);
      setMotorSpeed(2, 1, 0);
    } else {
      setMotorSpeed(1, 0, 255);
      setMotorSpeed(2, 0, 255);
    }
  } else if (horz > 0) {    // left
    if (vert > 0) {
      setMotorSpeed(1, 1, 125);
      setMotorSpeed(2, 1, 255);
    } else if (vert == 0) {
      setMotorSpeed(1, 0, 125);
      setMotorSpeed(2, 1, 125);
    } else {
      setMotorSpeed(1, 0, 125);
      setMotorSpeed(2, 0, 255);
    }
  } else {    // horz < 0    // right
    if (vert > 0) {
      setMotorSpeed(1, 1, 255);
      setMotorSpeed(2, 1, 125);
    } else if (vert == 0) {
      setMotorSpeed(1, 1, 125);
      setMotorSpeed(2, 0, 125);
    } else {
      setMotorSpeed(1, 0, 255);
      setMotorSpeed(2, 0, 125);
    }
  }
}

void setup() {
  pinMode(joystick_Vert_Pin, INPUT);
  pinMode(joystick_Horz_Pin, INPUT);

  // Motor pins setup
  pinMode(motor1_In1, OUTPUT);
  pinMode(motor1_In2, OUTPUT);
  pinMode(motor2_In1, OUTPUT);
  pinMode(motor2_In2, OUTPUT);

  // Encoder pins setup
  pinMode(encoder1_C, INPUT);    // INPUT_PULLUP
  pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  pinMode(encoder2_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  Serial.begin(115200); // For debugging output
  prevTime = millis();    // Initialize time

  Serial.println("---- setup finished ----");
};

void loop() {
  speedCtrl();

  // Calculate odometry for position and heading
  calculateOdometry();

  delay(100); // Adjust for desired loop rate
}
