const int DETECTION_LIMIT = 80; 

// Pinagem

// Motor direito
const int RIGHT_MOTOR_INT1_PIN = 4; 
const int RIGHT_MOTOR_INT2_PIN = 7; 
const int PWMA = 5; 

// Motor esquerdo
const int LEFT_MOTOR_INT3_PIN = 8; 
const int LEFT_MOTOR_INT4_PIN = 9; 
const int PWMB = 6; // PWM esquerdo

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
  uint8_t PWM; 
} MOTOR; 

MOTOR init_motor(uint8_t IN1, uint8_t IN2, uint8_t PWM) {
  MOTOR motor; 
  motor.IN1 = IN1; 
  motor.IN2 = IN2;
  motor.PWM = PWM; 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(PWM, OUTPUT); 
  return motor; 
}

void motor_foward(MOTOR motor, int speed) {
  digitalWrite(motor.IN1, LOW); 
  digitalWrite(motor.IN2, HIGH); 
  analogWrite(motor.PWM, speed); 
}

void motor_back(MOTOR motor, int speed) {
  digitalWrite(motor.IN1, HIGH); 
  digitalWrite(motor.IN2, LOW); 
  analogWrite(motor.PWM, speed); 
}

MOTOR right_motor = init_motor(RIGHT_MOTOR_INT1_PIN, RIGHT_MOTOR_INT2_PIN, PWMA); 
MOTOR left_motor = init_motor(LEFT_MOTOR_INT3_PIN, LEFT_MOTOR_INT4_PIN, PWMB); 

void move_robot_foward(int speed) {
  motor_foward(right_motor, speed); 
  motor_foward(left_motor, speed); 
}

void move_robot_backward(int speed) {
  motor_back(right_motor, speed); 
  motor_back(left_motor, speed); 
}

void turn_left(int speed) {
  motor_foward(right_motor, speed); 
  motor_back(left_motor, speed); 
}

void turn_right(int speed) {
  motor_back(right_motor, speed); 
  motor_foward(left_motor, speed); 
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
    move_robot_foward(255); 
  }

   // 2. Alinhamento frontal
  else if (front_right <= DETECTION_LIMIT) {
    Serial.println("Ajuste direita...");
    turn_right(255);
  }

  else if (front_left <= DETECTION_LIMIT) {
    Serial.println("Ajuste esquerda...");
    turn_left(255);
  }

  // 3. Alinhamento lateral
  else if (side_right <= DETECTION_LIMIT) {
    Serial.println("Virando pra direita (inimigo lateral)...");
    turn_right(255);
  }

  else if (side_left <= DETECTION_LIMIT) {
    Serial.println("Virando pra esquerda (inimigo lateral)...");
    turn_left(255);
  }

  // 4. Nenhum sensor detectou → procurar
  else {
    Serial.println("Procurando...");
    move_robot_foward(126);
  }
}

void loop() {
  sens_obs(); 

  if(digitalRead(LEFT_LINE_SENSOR_PIN) == LOW) {
    move_robot_backward(255); 
    Serial.println("tras..."); 
    delay(200); 
    turn_right(255); 
    Serial.println("Direita..."); 
    delay(250); 
  }

  if(digitalRead(RIGHT_LINE_SENSOR_PIN) == LOW) {
    move_robot_backward(255); 
    Serial.println("tras..."); 
    delay(200); 
    turn_left(255); 
    Serial.println("Esquerda..."); 
    delay(250); 
  }

}
