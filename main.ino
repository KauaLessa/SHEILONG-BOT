const int DETECTION_LIMIT = 80; 

// Pinagem

// Motor direito
const int RIGHT_MOTOR_AI1_PIN = 4; 
const int RIGHT_MOTOR_AI2_PIN = 7; 

// Motor esquerdo
const int LEFT_MOTOR_BI1_PIN = 8; 
const int LEFT_MOTOR_BI2_PIN = 9; 

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

typedef struct MOTOR {
  uint8_t IN1; 
  uint8_t IN2; 
} MOTOR; 

MOTOR init_motor(uint8_t IN1, uint8_t IN2) {
  MOTOR motor; 
  motor.IN1 = IN1; 
  motor.IN2 = IN2; 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  return motor; 
}

void motor_foward(MOTOR motor) {
  digitalWrite(motor.IN1, HIGH); 
  digitalWrite(motor.IN2, LOW); 
}

void motor_back(MOTOR motor) {
  digitalWrite(motor.IN1, LOW); 
  digitalWrite(motor.IN2, HIGH); 
}

MOTOR right_motor = init_motor(RIGHT_MOTOR_AI1_PIN, RIGHT_MOTOR_AI2_PIN); 
MOTOR left_motor = init_motor(LEFT_MOTOR_BI1_PIN, LEFT_MOTOR_BI2_PIN); 

void move_robot_foward() {
  motor_foward(right_motor); 
  motor_foward(left_motor); 
}

void move_robot_backward() {
  motor_back(right_motor); 
  motor_back(left_motor); 
}

void turn_left() {
  motor_foward(right_motor); 
  motor_back(left_motor); 
}

void turn_right() {
  motor_back(right_motor); 
  motor_foward(left_motor); 
}

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


void sens_obs() {
  int front_right = getDistance(OBS1_SENSOR_PIN); 
  int front_left  = getDistance(OBS2_SENSOR_PIN); 
  int side_right  = getDistance(OBS3_SENSOR_PIN); 
  int side_left   = getDistance(OBS4_SENSOR_PIN); 

  // 1. Ataque direto (inimigo na frente)
  if (front_right <= DETECTION_LIMIT && front_left <= DETECTION_LIMIT) {
    Serial.println("ATAQUE!!!");
    move_robot_foward(); 
  }

  // 2. Alinhamento lateral
  else if (side_right <= DETECTION_LIMIT) {
    Serial.println("Virando pra direita (inimigo lateral)...");
    turn_right();
  }

  else if (side_left <= DETECTION_LIMIT) {
    Serial.println("Virando pra esquerda (inimigo lateral)...");
    turn_left();
  }

  // 3. Alinhamento frontal
  else if (front_right <= DETECTION_LIMIT) {
    Serial.println("Ajuste direita...");
    turn_right();
  }

  else if (front_left <= DETECTION_LIMIT) {
    Serial.println("Ajuste esquerda...");
    turn_left();
  }

  // 4. Nenhum sensor detectou → procurar
  else {
    Serial.println("Procurando...");
    move_robot_foward();
  }
}

void loop() {
  sens_obs(); 

  if(digitalRead(LEFT_LINE_SENSOR_PIN) == LOW) {
    move_robot_backward(); 
    Serial.println("tras..."); 
    delay(200); 
    turn_right(); 
    Serial.println("Direita..."); 
    delay(250); 
  }

  if(digitalRead(RIGHT_LINE_SENSOR_PIN) == LOW) {
    move_robot_backward(); 
    Serial.println("tras..."); 
    delay(200); 
    turn_left(); 
    Serial.println("Esquerda..."); 
    delay(250); 
  }

}
