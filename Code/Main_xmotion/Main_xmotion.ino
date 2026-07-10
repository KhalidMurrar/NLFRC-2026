// pid
struct PID {
  float Kp;
  float Ki;
  float Kd;

  float error;
  float lastError;
  float integral;
  float derivative;

  float output;

  float integralLimit;
  float outputLimit;

  unsigned long lastTime;
  float dt;
};
PID mainPID;

// flags
bool running;
bool calibrate_flag;

bool no_line_detected = false;
bool has_detected_all_black = false;
bool all_lines_detected = false;
bool intersection = false;
unsigned long last_time_all_detected;

int stop_time_delay = 100;

// speeds
int MIN_SPEED;
int BASE_SPEED;
int MAX_SPEED;
int SEARCH_SPEED;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// motors
#define LM_DIR 13
#define LM_PWM 10
#define RM_DIR 5
#define RM_PWM 9

// xline
#define S0 1
#define S1 14
#define S2 12
#define S3 4
#define Sens 0
const int selectPins[] = { S0, S1, S2, S3 };

#define LED_BUILTIN A3
#define buzzer A0
#define BUTTON_PIN 11
#define DIP1 6
#define DIP2 7
#define DIP3 8


bool button_state = false;

unsigned long startupTime = 0;

void setup() {
  // motors
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  
  Serial.begin(9600);

  

  // xline
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(Sens, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(DIP3, INPUT_PULLUP);

  while(true) {
    button_state = digitalRead(BUTTON_PIN);
    Serial.println("waiting");
    if (!button_state) {
      running = true;
      break;
    }
  }

  if (!digitalRead(DIP1)) {
    MIN_SPEED = 20;
    BASE_SPEED = 60;
    MAX_SPEED = 100;
    SEARCH_SPEED = 80;

    // calibrate PID
    mainPID.Kp = 12;
    mainPID.Ki = 0.0;
    mainPID.Kd = 0.85;
    Serial.println("1");
  } else if (!digitalRead(DIP2)) {
    MIN_SPEED =20;
    BASE_SPEED = 80;
    MAX_SPEED = 140;
    SEARCH_SPEED = 100;
    // calibrate PID
    mainPID.Kp = 15.3;
    mainPID.Ki = 0.0;
    mainPID.Kd = 1.3;
    Serial.println("2");
  } else if (!digitalRead(DIP3)) {
    MIN_SPEED = 20;
    BASE_SPEED = 60;
    MAX_SPEED = 200;
    SEARCH_SPEED = 80;
    // calibrate PID
    mainPID.Kp = 0.85;
    mainPID.Ki = 0.0;
    mainPID.Kd = 0.85;
    Serial.println("3");
  }
  Serial.println("Started");
  startupTime = millis();
}

void loop() {
  int16_t sensor_values[15];
  int bottom_Sensor;
  readSensor(sensor_values, bottom_Sensor);

  if (all_lines_detected) {
    Serial.println("All lines detected");
    controlMotors(0, 0);
    running = false;
  } else if (running) {
    float error = calculateError(sensor_values, mainPID, bottom_Sensor);

    if (no_line_detected) {  // no line
      if (error > 0) {
        controlMotors(SEARCH_SPEED, -SEARCH_SPEED);
      } else {
        controlMotors(-SEARCH_SPEED, SEARCH_SPEED);
      }
    } else if (intersection) {
       digitalWrite(LED_BUILTIN, HIGH);
       //controlMotors(MAX_SPEED, MIN_SPEED);
    } else {  // line
      calculatePID(mainPID, error);
    }
  } else {  // not running
    digitalWrite(LED_BUILTIN, LOW);
    mainPID.lastError = 0;
    mainPID.integral = 0;
    controlMotors(0, 0);
  }
}

void calculatePID(PID& pid, float newError) {
  unsigned long now = millis();
  pid.dt = (now - pid.lastTime) / 1000.0;

  if (pid.dt == 0) pid.dt = 0.001;

  pid.error = newError;

  pid.integral += pid.error * pid.dt;

  // if the output is a big value it means he robot is lost
  if (abs(pid.output) > 5000 || abs(pid.error <= 1)) {
    pid.integral = 0;
  }

  pid.derivative = (pid.error - pid.lastError) / pid.dt;

  pid.output = (pid.Kp * pid.error) + (pid.Kd * pid.derivative) + (pid.Ki * pid.integral);

  pid.lastError = pid.error;
  pid.lastTime = now;

  currentRightSpeed = constrain(BASE_SPEED + pid.output, MIN_SPEED, MAX_SPEED);
  currentLeftSpeed = constrain(BASE_SPEED - pid.output, MIN_SPEED, MAX_SPEED);

  controlMotors(currentRightSpeed, currentLeftSpeed);
}

void readSensor(int16_t* values, int &bottomSensor) {
  // read top array
  for (int i = 0; i < 15; i++) {
    digitalWrite(S0, (i >> 0) & 1);
    digitalWrite(S1, (i >> 1) & 1);
    digitalWrite(S2, (i >> 2) & 1);
    digitalWrite(S3, (i >> 3) & 1);
    delayMicroseconds(20);

    values[i] = digitalRead(Sens);
  }
  // read bottom sensor
  digitalWrite(S0, 1);
  digitalWrite(S1, 1);
  digitalWrite(S2, 1);
  digitalWrite(S3, 1);
  delayMicroseconds(20);

  bottomSensor = digitalRead(Sens);
}

float calculateError(int16_t* sensor_values, PID& pid, bool backSensor) {
  float weights[]{ -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7 };
  float error = 0;
  int active_sensors = 0;
  int white_count = 0;
  bool inverted;

  // if sensor sees black add its reading to the list
  // 1 is white 0 is black
  for (int i=0; i<15; i++) {
    white_count += sensor_values[i];
  }

  
  if (white_count <= 10 && backSensor) inverted = true;
  else if (white_count >= 10) inverted = false;
  

  for (int i = 0; i < 15; i++) {
    bool line_detected = inverted ? sensor_values[i] : !sensor_values[i];
    if (line_detected) {
      error += weights[i];
      active_sensors++;
    }
  }

  Serial.print("Active count; "); Serial.print(active_sensors);
  Serial.print(" | White count: "); Serial.print(white_count);
  Serial.print(" | Sensors: ");
  for (int i = 0; i < 15; i++) {
    Serial.print(sensor_values[i]);
  }

  Serial.println();

  // if all sensors see white reset the error
  // if the last error was greater than 0 turn right
  if (active_sensors == 0) {
    no_line_detected = true;
    all_lines_detected = false;
    return (pid.lastError > 0) ? 5.0 : -5.0;
  } else if (active_sensors == 15 && !backSensor) {
    if (!has_detected_all_black) {
      has_detected_all_black = true;
      last_time_all_detected = millis();
    } else if (millis() - last_time_all_detected > stop_time_delay) {
      all_lines_detected = true;
    }

    no_line_detected = false;
    return error / active_sensors;
  } else if (active_sensors >= 8) {
    intersection = true;
    return error / active_sensors;
  }
  else {
    has_detected_all_black = false;
    all_lines_detected = false;
    no_line_detected = false;
    intersection = false;
    return error / active_sensors;
  }
}


void controlMotors(int RightMotorPwm, int LeftMotorPwm) {
  if (RightMotorPwm <= 0) {
    RightMotorPwm = abs(RightMotorPwm);
    digitalWrite(RM_DIR, LOW);
    analogWrite(RM_PWM, RightMotorPwm);
  }
  else {
    digitalWrite(RM_DIR, HIGH);
    analogWrite(RM_PWM, RightMotorPwm);
  }
  if (LeftMotorPwm <= 0) {
    LeftMotorPwm = abs(LeftMotorPwm);
    digitalWrite(LM_DIR, LOW);
    analogWrite(LM_PWM, LeftMotorPwm);
  }
  else {
    digitalWrite(LM_DIR, HIGH);
    analogWrite(LM_PWM, LeftMotorPwm);
  }
}
