const int DETECTION_THRESHOLD = 150; 

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

// Armazenamento da leitura dos sensores

int front_right;
int front_left;
int side_right;
int side_left;
bool line_left;
bool line_right;

unsigned long retreat_timer = 0; // timer de retirada

const int NUM_READINGS = 5; // Leituras (filtro de média móvel)

typedef enum { 
  ATTACK,
  ALIGN_FRONT_LEFT, 
  ALIGN_FRONT_RIGHT,
  ALIGN_LAT_LEFT,
  ALIGN_LAT_RIGHT, 
  RETREAT_LEFT,
  RETREAT_RIGHT
} State;

State current_state = ATTACK; 
State previous_state; 

typedef enum {LEFT, RIGHT} Direction; 

typedef struct {
  int idx; 
  int total;
  int avg; 
  int readings[NUM_READINGS]; 
} MovingAverage; 

typedef struct {
  uint8_t IN1; 
  uint8_t IN2;
  uint8_t PWM; 
} Motor;

MovingAverage initMV() {
  MovingAverage mv; 
  mv.idx = 0; 
  mv.total = 0; 
  mv.avg = 0; 

  for (int i = 0; i < NUM_READINGS; i++)
    mv.readings[i] = 0; 

  return mv; 
}

int calculateMV(MovingAverage *mv, int last_reading) {
  mv->total = mv->total - mv->readings[mv->idx]; 

  mv->readings[mv->idx] = last_reading;   

  mv->total += last_reading;

  mv->avg = mv->total / NUM_READINGS; 

  mv->idx = (mv->idx + 1) % NUM_READINGS; 

  return mv->avg; 
}
 
Motor init_motor(uint8_t IN1, uint8_t IN2, uint8_t PWM) {
  Motor motor; 
  motor.IN1 = IN1; 
  motor.IN2 = IN2;
  motor.PWM = PWM; 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(PWM, OUTPUT); 
  return motor; 
}

void motor_forward(Motor motor, int speed) {
  digitalWrite(motor.IN1, LOW); 
  digitalWrite(motor.IN2, HIGH); 
  analogWrite(motor.PWM, speed); 
}

void motor_back(Motor motor, int speed) {
  digitalWrite(motor.IN1, HIGH); 
  digitalWrite(motor.IN2, LOW); 
  analogWrite(motor.PWM, speed); 
}

Motor right_motor = init_motor(RIGHT_MOTOR_INT1_PIN, RIGHT_MOTOR_INT2_PIN, PWMA); 
Motor left_motor = init_motor(LEFT_MOTOR_INT3_PIN, LEFT_MOTOR_INT4_PIN, PWMB); 

void move_robot_forward(int speed) {
  motor_forward(right_motor, speed); 
  motor_forward(left_motor, speed); 
}

void move_robot_backward(int speed) {
  motor_back(right_motor, speed); 
  motor_back(left_motor, speed); 
}

void turn_left(int speed) {
  motor_forward(right_motor, speed); 
  motor_back(left_motor, speed); 
}

void turn_right(int speed) {
  motor_back(right_motor, speed); 
  motor_forward(left_motor, speed); 
}

bool detect_obs(int reading) {
  return (reading >= DETECTION_THRESHOLD); 
}

bool detect_border(uint8_t line_pin) {
  return (digitalRead(line_pin) == LOW); 
}

// Filtros de média móvel

MovingAverage mv_frontR = initMV(); 
MovingAverage mv_frontL = initMV();
MovingAverage mv_latR   = initMV();
MovingAverage mv_latL   = initMV();

void read_sens() {
  front_right  = analogRead(OBS1_SENSOR_PIN); 
  front_left   = analogRead(OBS2_SENSOR_PIN); 
  side_right   = analogRead(OBS3_SENSOR_PIN); 
  side_left    = analogRead(OBS4_SENSOR_PIN);
  line_left    = detect_border(LEFT_LINE_SENSOR_PIN); 
  line_right   = detect_border(RIGHT_LINE_SENSOR_PIN);
}

void apply_mv_filter() {
  front_right  = calculateMV(&mv_frontR, front_right); 
  front_left   = calculateMV(&mv_frontL, front_left); 
  side_left    = calculateMV(&mv_latL, side_left); 
  side_right   = calculateMV(&mv_latR, side_right); 
}

State next_state() {

  if (detect_obs(front_left) && detect_obs(front_right)) return ATTACK;

  if (line_left) return RETREAT_LEFT; 

  if (line_right) return RETREAT_RIGHT; 

  if (detect_obs(front_right)) return ALIGN_FRONT_RIGHT; 

  if (detect_obs(front_left)) return ALIGN_FRONT_LEFT; 

  if (detect_obs(side_left)) return ALIGN_LAT_LEFT; 

  if (detect_obs(side_right)) return ALIGN_LAT_RIGHT; 

  return ATTACK; 
}

void run_retreat(Direction dir) {
  unsigned long now = millis();

  const unsigned long BACKWARD_TIME = 300;
  const unsigned long TURN_TIME = 250;

  if (retreat_timer == 0) retreat_timer = now;

  unsigned long elapsed = now - retreat_timer;

  if (elapsed < BACKWARD_TIME) {
    move_robot_backward(255);
  }
  else if (elapsed < BACKWARD_TIME + TURN_TIME) {
    if (dir == LEFT) turn_left(255); 
    else turn_right(255);
  }
  else {
    retreat_timer = 0;
  }
}

void run_state() {
  switch(current_state) {
    case(ATTACK):
      move_robot_forward(255);
    break;

    case(RETREAT_LEFT):
      run_retreat(LEFT);
    break; 

    case(RETREAT_RIGHT):
      run_retreat(RIGHT); 
    break; 

    case(ALIGN_FRONT_RIGHT):
      turn_right(255); 
    break; 

    case(ALIGN_FRONT_LEFT):
      turn_left(255); 
    break; 

    case(ALIGN_LAT_LEFT):
      turn_left(255); 
    break; 

    case(ALIGN_LAT_RIGHT):
      turn_right(255); 
    break; 

    default:
      move_robot_forward(255); 
  }
}

void setup() {
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  delay(5000); 
}

void loop() {
  previous_state = current_state; 

  read_sens();
  delay(5); 
  apply_mv_filter();

  current_state = next_state(); 

  if(previous_state != current_state) {
    retreat_timer = 0; 
  }

  run_state(); 
}
