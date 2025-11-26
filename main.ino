#include <SparkFun_TB6612.h>

const int DETECTION_LIMIT = 80; 

// Pinagem

// Motor direito
const int RIGHT_MOTOR_AI1_PIN = 4; 
const int RIGHT_MOTOR_AI2_PIN = 7; 
const int RIGHT_MOTOR_PWM_PIN = 6; 

// Motor esquerdo
const int LEFT_MOTOR_BI1_PIN = 8; 
const int LEFT_MOTOR_BI2_PIN = 9; 
const int LEFT_MOTOR_PWM_PIN = 5; 

// Sensores de linha
const int RIGHT_LINE_SENSOR_PIN = 2; 
const int LEFT_LINE_SENSOR_PIN = 3; 

// Sensores de obstáculo

// Sensores frontais
const int OBS1_SENSOR_PIN = A4; // Direita
const int OBS2_SENSOR_PIN = A3; // Esquerda

// Sensores laterais
const int OBS3_SENSOR_PIN = A5; // Direita
const int OBS4_SENSOR_PIN = A6; // Esquerda

Motor right_motor = Motor(
  RIGHT_MOTOR_AI1_PIN, 
  RIGHT_MOTOR_AI2_PIN, 
  RIGHT_MOTOR_PWM_PIN,
  1,
  A7
);

Motor left_motor = Motor(
  LEFT_MOTOR_BI1_PIN,
  LEFT_MOTOR_BI2_PIN,
  LEFT_MOTOR_PWM_PIN,
  1,
  A7
);

void setup() {
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  delay(5000); 
}

int getDistance(int sensor_pin) {
  float voltage = analogRead(sensor_pin) * (5.0 / 1023.0); // Convert analog value to voltage
  float distanceCM = 27.86 * pow(voltage, -1.15);     // Calculate distance in cm
  return distanceCM; 
}

void turnLeft(int speed) {
  right_motor.drive(speed); 
  left_motor.drive(-speed); 
}

void turnRight(int speed) {
  right_motor.drive(-speed); 
  left_motor.drive(speed); 
}

void sens_obs() {
  int front_right = getDistance(OBS1_SENSOR_PIN); 
  int front_left  = getDistance(OBS2_SENSOR_PIN); 
  int side_right  = getDistance(OBS3_SENSOR_PIN); 
  int side_left   = getDistance(OBS4_SENSOR_PIN); 

  // 1. Ataque direto (inimigo na frente)
  if (front_right <= DETECTION_LIMIT && front_left <= DETECTION_LIMIT) {
    Serial.println("ATAQUE!!!");
    forward(right_motor, left_motor, 255);
  }

  // 2. Alinhamento lateral
  else if (side_right <= DETECTION_LIMIT) {
    Serial.println("Virando pra direita (inimigo lateral)...");
    turnRight(255);
  }

  else if (side_left <= DETECTION_LIMIT) {
    Serial.println("Virando pra esquerda (inimigo lateral)...");
    turnLeft(255);
  }

  // 3. Alinhamento frontal
  else if (front_right <= DETECTION_LIMIT) {
    Serial.println("Ajuste direita...");
    turnRight(200);
  }

  else if (front_left <= DETECTION_LIMIT) {
    Serial.println("Ajuste esquerda...");
    turnLeft(200);
  }

  // 4. Nenhum sensor detectou → procurar
  else {
    Serial.println("Procurando...");
    forward(right_motor, left_motor, 120);
  }
}

void loop() {
  sens_obs(); 

  if(digitalRead(LEFT_LINE_SENSOR_PIN) == LOW) {
    back(right_motor, left_motor, 255); 
    Serial.println("tras..."); 
    delay(200); 
    turnRight(255); 
    Serial.println("Direita..."); 
    delay(250); 
  }

  if(digitalRead(RIGHT_LINE_SENSOR_PIN) == LOW) {
    back(right_motor, left_motor, 255); 
    Serial.println("tras..."); 
    delay(200); 
    turnLeft(255); 
    Serial.println("Esquerda..."); 
    delay(250); 
  }

}
