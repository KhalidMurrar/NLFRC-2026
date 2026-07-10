#include <WiFi.h>

void dmpDataReady();

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
unsigned long last_time_all_detected;

int stop_time_delay = 250;

// speeds
volatile int MIN_SPEED = 100;
volatile int BASE_SPEED = 40;
volatile int MAX_SPEED = 60;
volatile int SEARCH_SPEED = 80;

bool manualMode = false;
volatile int manualRightSpeed = 0;
volatile int manualLeftSpeed = 0;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// WiFi config
const char* ssid = "KhalidOPPO";
const char* password = "khalid2007";
WiFiServer server(80);
WiFiClient client;

unsigned long last_send_time = 0;
const unsigned long send_interval = 10;

// motors
#define LEFT_MOTOR_FORWARD 14
#define LEFT_MOTOR_BACKWARD 27
#define RIGHT_MOTOR_FORWARD 26
#define RIGHT_MOTOR_BACKWARD 13

// xline
#define S0 18
#define S1 19
#define S2 4
#define S3 16
#define Sens 17
const int selectPins[] = { S0, S1, S2, S3 };

#define LED_BUILTIN 2
#define BUTTON_PIN 32

unsigned long startupTime = 0;

void initWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void setup() {
  // motors
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);

  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  
  Serial.begin(115200);
  initWiFi();

  // calibrate PID
  mainPID.Kp = 12.0;
  mainPID.Ki = 0.0;
  mainPID.Kd = 0.85;

  // xline
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(Sens, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  startupTime = millis();
}

void loop() {
  int16_t sensor_values[15];
  bool bottomSensor;
  readSensor(sensor_values, bottomSensor);

  if (millis() - last_send_time >= send_interval) {
    sendLineSensorData(sensor_values);
    sendMotorData();
    last_send_time = millis();
  }

  checkWiFi();

  setMotorSpeed('R', 100);
  setMotorSpeed('L', 100);

  // if (manualMode) {
  //   setMotorSpeed('R', manualRightSpeed);
  //   setMotorSpeed('L', manualLeftSpeed);
  // } else if (all_lines_detected) {
  //   setMotorSpeed('R', 0);
  //   setMotorSpeed('L', 0);
  //   running = false;
  // } else if (running) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   float error = calculateError(sensor_values, mainPID);

  //   if (no_line_detected) {  // no line
  //     if (error > 0) {
  //       setMotorSpeed('R', SEARCH_SPEED);
  //       setMotorSpeed('L', -SEARCH_SPEED);
  //     } else {
  //       setMotorSpeed('R', -SEARCH_SPEED);
  //       setMotorSpeed('L', SEARCH_SPEED);
  //     }
  //   } else {  // line
  //     calculatePID(mainPID, error);
  //   }
  // } else {  // not running
  //   digitalWrite(LED_BUILTIN, LOW);
  //   mainPID.lastError = 0;
  //   mainPID.integral = 0;
  //   setMotorSpeed('R', 0);
  //   setMotorSpeed('L', 0);
  // }

  vTaskDelay(1);
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

  setMotorSpeed('R', currentRightSpeed);
  setMotorSpeed('L', currentLeftSpeed);
}

void readSensor(int16_t* values, bool bottomSensor) {
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

float calculateError(int16_t* sensor_values, PID& pid) {
  float weights[]{ -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7 };
  float error = 0;
  int active_sensors = 0;

  // if sensor sees black add its reading to the list
  // 1 is white 0 is black
  for (int i = 0; i < 15; i++) {
    if (!sensor_values[i]) {
      error += weights[i];
      active_sensors++;
    }
  }

  // if all sensors see white reset the error
  // if the last error was greater than 0 turn right
  if (active_sensors == 0) {
    no_line_detected = true;
    all_lines_detected = false;
    return (pid.lastError > 0) ? 1.0 : -1.0;
  } else if (active_sensors == 15) {
    /*
    ***********************
    ***********************
    ***********************
             WTF??
    ***********************
    ***********************
    ***********************
    */
    if (!has_detected_all_black) {
      has_detected_all_black = true;
      last_time_all_detected = millis();
    } else if (millis() - last_time_all_detected > stop_time_delay) {
      all_lines_detected = true;
    }

    no_line_detected = false;
    return error / active_sensors;
  } else {
    has_detected_all_black = false;
    all_lines_detected = false;
    no_line_detected = false;
    return error / active_sensors;
  }
}

void setMotorSpeed(char side, int speed) {
  if (side == 'R') {
    if (speed > 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, abs(speed));
      analogWrite(RIGHT_MOTOR_BACKWARD, LOW);
    } else if (speed < 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, LOW);
      analogWrite(RIGHT_MOTOR_BACKWARD, abs(speed));
    } else {
      analogWrite(RIGHT_MOTOR_FORWARD, LOW);
      analogWrite(RIGHT_MOTOR_BACKWARD, LOW);
    }
  } else if (side == 'L') {
    if (speed > 0) {
      analogWrite(LEFT_MOTOR_FORWARD, abs(speed));
      analogWrite(LEFT_MOTOR_BACKWARD, LOW);
    } else if (speed < 0) {
      analogWrite(LEFT_MOTOR_FORWARD, LOW);
      analogWrite(LEFT_MOTOR_BACKWARD, abs(speed));
    } else {
      analogWrite(LEFT_MOTOR_FORWARD, LOW);
      analogWrite(LEFT_MOTOR_BACKWARD, LOW);
    }
  }
}

void checkWiFi() {
  if (!client) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
    }
  }

  if (client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();

    if (command.startsWith("PID:")) {
      parsePID(command);
    } else if (command.startsWith("VALUES:")) {
      parseValues(command);
    } else if (command = "RESET") {
      running = false;
      manualMode = false;
      client.println("STATUS: Reset complete");
    } else if (command == "START") {
      running = true;
      manualMode = false;
      client.println("STATUS: Started");
    } else if (command == "STOP") {
      running = false;
      manualMode = false;
      client.println("STATUS: Stopped");
    } else if (command = "CALIBRATE") {
      calibrate_flag = true;
      client.println("STATUS: Calibrating...");
    } else if (command.indexOf(',') != -1) {
      int commaIndex = command.indexOf(',');
      int l_speed = command.substring(0, commaIndex).toInt();
      int r_speed = command.substring(commaIndex+1).toInt();

      currentLeftSpeed = l_speed;
      currentRightSpeed = r_speed;
      manualLeftSpeed = l_speed;
      manualRightSpeed = r_speed;
      manualMode = true;
      running = false;
      client.println("STATUS: Manual speed set");
      sendMotorData();
    } else if (command.startsWith("SET_ANGLE ")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex != -1) {
        client.println("STATUS: Rotating to target angle");
      }
    } else if (command.startsWith("MAX ")) {
      int speed = command.substring(4).toInt();
      MAX_SPEED = speed;
      client.print("STATUS: MAX_SPEED set to");
      client.println(MAX_SPEED);
    } else if (command.startsWith("MIN ")) {
      int speed = command.substring(4).toInt();
      MIN_SPEED = speed;
      client.print("STATUS: MIN_SPED set to");
      MIN_SPEED = speed;
      client.print("STATUS: MIN_SPEED set to ");
      client.println(MIN_SPEED);
    } else if (command.startsWith("BASE ")) {
      int speed = command.substring(5).toInt();
      BASE_SPEED = speed;
      client.print("STATUS: BASE_SPEED set to ");
      client.println(BASE_SPEED);
    } else if (command.startsWith("STOP_TIME")) {
      int time = command.substring(5).toInt();
      stop_time_delay = time;
      client.print("STATUS: stop_time_delay set to ");
      client.println(stop_time_delay);
    }
  }
}

void parsePID(String command) {
  command = command.substring(4);
  int comma1 = command.indexOf(',');
  int comma2 = command.lastIndexOf(',');

  if (comma1 != -1 && comma2 != -1) {
    mainPID.Kp = command.substring(0, comma1).toFloat();
    mainPID.Ki = command.substring(comma1+1, comma2).toFloat();
    mainPID.Kd = command.substring(comma2+1).toFloat();
    client.print("STATUS: PID Updated - KP:");
    client.print(mainPID.Kp);
    client.print(" KI:");
    client.print(mainPID.Ki);
    client.print(" KD:");
    client.println(mainPID.Kd);
  }
}

void parseValues(String command) {
    command = command.substring(7); // Remove "VALUES:" prefix

    int lastIndex = 0;

    // The loop runs 7 times for 7 values (index 0 to 6)
    for (int i = 0; i < 6; i++) {
        // Find the next slash
        int commaIndex = command.indexOf(',', lastIndex);
        
        String valueStr;

        // For the first 6 values, substring to the comma.
        // For the last value, take the rest of the string.
        if (i < 5) {
            // If a comma is not found before the last value, the format is wrong.
            if (commaIndex == -1) {
                client.println("STATUS: ERROR - Invalid VALUES format. Not enough values.");
                return;
            }
            valueStr = command.substring(lastIndex, commaIndex);
            lastIndex = commaIndex + 1; // Move past the comma for the next search
        } else {
            valueStr = command.substring(lastIndex);
        }

        // Use a switch statement for cleaner assignment
        switch (i) {
            case 0: mainPID.Kp = valueStr.toFloat(); break;
            case 1: mainPID.Ki = valueStr.toFloat(); break;
            case 2: mainPID.Kd = valueStr.toFloat(); break;
            case 3: BASE_SPEED = valueStr.toInt(); break;   // Assigns the 4th value
            case 4: MAX_SPEED = valueStr.toInt(); break;    // Assigns the 5th value
            case 5: SEARCH_SPEED = valueStr.toInt(); break; // CORRECTLY assigns the 6th value
        }
    }

    // Print confirmation status
    client.print("STATUS: Values Updated - KP:");
    client.print(mainPID.Kp);
    client.print(" KI:");
    client.print(mainPID.Ki);
    client.print(" KD:");
    client.print(mainPID.Kd);
    client.print(" MIN:");
    client.print(MIN_SPEED);
    client.print(" BASE:");
    client.print(BASE_SPEED);
    client.print(" MAX:");
    client.print(MAX_SPEED);
    client.print(" SEARCH:");
    client.println(SEARCH_SPEED);
}

void sendLineSensorData(int16_t* values) {
  if (!client) return;
  client.print("LINE:");
  for (int i=0; i<15; i++) {
    client.print(values[i]);
    if (i < 14) client.print(",");
  }
  client.println();
}

void sendMotorData() {
    if (!client) return;
    client.print("MOTORS:");
    client.print(currentLeftSpeed);
    client.print(",");
    client.print(currentRightSpeed);
    client.print(",");
    client.print(manualLeftSpeed);
    client.print(",");
    client.println(manualRightSpeed);
}

void IRAM_ATTR dmpDataRead() {
  
}
