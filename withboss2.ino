//with the boss

#include <Arduino.h>

// Arduino Pins and Constant Values
const int EN_PIN = 5;           // enable, active when LOW
const int DIR_PIN = 3;          // direction, (DIR_PIN, isClockwise ? LOW : HIGH)
const int STEP_PIN = 4;         // step, for-loop step angles

const int TOP_SENSOR_PIN = 12;
const int BOTTOM_SENSOR_PIN = 13;

const int SENSOR_BACK_STEP = 22;
const float STEP_ANGLE = 1.8;
const int STEP_DELAY = 2500;     // Initial step delay value 64

const int RELAY_PIN[] = {2, 6, 7, 8};

const int MAX_ROT = 50;

float _step_current_position = 0;

int _step_delay = STEP_DELAY;    // Sensor response delay

enum class MotorState { STOPPED, MOVING_CW, MOVING_CCW };
MotorState _motorState = MotorState::STOPPED;

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(TOP_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BOTTOM_SENSOR_PIN, INPUT_PULLUP);
  homePosition();  // Initialize the motor at startup

  for (int i = 0; i < sizeof(RELAY_PIN) / sizeof(RELAY_PIN[0]); i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    digitalWrite(RELAY_PIN[i], LOW);
  }

  Serial.begin(115200);
  Serial.flush();
}


void motorStep(bool isClockwise, int steps) {
  for (int i = 0; i < steps; i++) {
    int step_current_position = _step_current_position; // Store the current position

    digitalWrite(DIR_PIN, isClockwise ? HIGH : LOW);
    delayMicroseconds(_step_delay);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(_step_delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(_step_delay);

    if (isClockwise)
      //step_current_position--;
      step_current_position--;

    else
      //step_current_position++;
      step_current_position++;


    _step_current_position = step_current_position;
    Serial.print("motorStep,_Current  " + String(_step_current_position)  +  "  current  " + String(step_current_position) +  ",ED\r\n");

  }
}


/*
void motorStep(bool isClockwise, int steps) {
  for (int i = 0; i < steps; i++) {
    int step_current_position = _step_current_position; // Store the current position

    digitalWrite(DIR_PIN, isClockwise ? HIGH : LOW); 
    delayMicroseconds(_step_delay);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(_step_delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(_step_delay);

    if (isClockwise) {
      step_current_position--;

      // Check for top sensor interruption in clockwise direction
      if (digitalRead(TOP_SENSOR_PIN) == LOW) {
        stopMotor();
        delay(1000);  // Stop for a second
        motorStep(false, SENSOR_BACK_STEP);  // Rotate 7 degrees CW
        stopMotor();
      }
    } else {
      step_current_position++;

      // Check for bottom sensor interruption in counter-clockwise direction
      if (digitalRead(BOTTOM_SENSOR_PIN) == LOW) {
        stopMotor();
        delay(1000);  // Stop for a second
        motorStep(true, SENSOR_BACK_STEP);  // Rotate 7 degrees CCW
        stopMotor();
      }
    }

    _step_current_position = step_current_position;
    Serial.print("motorStep,_Current  " + String(_step_current_position)  +  "  current  " + String(step_current_position) +  ",ED\r\n");

  }
}

*/

void stopMotor() {
  digitalWrite(EN_PIN, LOW);
  _motorState = MotorState::STOPPED;
}

String inputValues(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


//"ST,0,MOTOR,0,30,90,ED\r\n"

void motorControl(String motorCommand) {
  int  currentPos =_step_current_position;

  int targetAngle = inputValues(motorCommand, ',', 4).toInt();
  int speedPercent = inputValues(motorCommand, ',', 5).toInt();

  //int step_target_position = static_cast<int>(targetAngle / STEP_ANGLE) + SENSOR_BACK_STEP;
  int step_target_position = static_cast<int>(targetAngle / STEP_ANGLE);


  bool isClockwise = (currentPos < step_target_position);
  Serial.print("motorControl,Current  " + String(currentPos)  +  "  target  " + String(step_target_position) +  "  CW  " + String(isClockwise) + ",ED\r\n");


  int speed = map(speedPercent, 0, 100, STEP_DELAY, 1500);

  //_motorState = isClockwise ? MotorState::MOVING_CW : MotorState::MOVING_CCW;

  while (currentPos != step_target_position) {
    bool stopSensorTop = digitalRead(TOP_SENSOR_PIN) == LOW;
    bool stopSensorBottom = digitalRead(BOTTOM_SENSOR_PIN) == LOW;

    /*if (stopSensorTop || stopSensorBottom) {
      delay(speed);
      stopMotor();

      if (isClockwise) {
        motorStep(true, SENSOR_BACK_STEP); // Rotate 7 degrees CW
      } else {
        motorStep(false, SENSOR_BACK_STEP);  // Rotate 7 degrees CCW
      }

      delay(speed);
      return;
    }*/

    motorStep(isClockwise, 1);
  }

  Serial.print("ST,0,RETMOVE,OK," + String(step_target_position) + "," + String(speedPercent) + ",ED\r\n");

}

void relayControl(String relayCommand) {
  String relayName = inputValues(relayCommand, ',', 2);
  relayName.toUpperCase();
  int relayState = inputValues(relayCommand, ',', 3).toInt();

  int relayPinIndex = -1;

  if (relayName.equals("PRINTER")) {
    relayPinIndex = 0;
  } else if (relayName.equals("DISPENSER")) {
    relayPinIndex = 1;
  } else if (relayName.equals("CARD_READER")) {
    relayPinIndex = 2;
  } else if (relayName.equals("PASSPORT_READER")) {
    relayPinIndex = 3;
  } else {
    Serial.print("ST,0," + relayName + ",ERR_INVALID_COMMAND,ED\r\n");
    return;
  }

  if (relayState == 1 && digitalRead(RELAY_PIN[relayPinIndex]) == LOW) {
    digitalWrite(RELAY_PIN[relayPinIndex], HIGH);
  } else if (relayState == 0 && digitalRead(RELAY_PIN[relayPinIndex]) == HIGH) {
    digitalWrite(RELAY_PIN[relayPinIndex], LOW);
  }

  Serial.print("ST,0," + relayName + ",OK," + String(digitalRead(RELAY_PIN[relayPinIndex])) + ",ED\r\n");
}

void homePosition() {
  int stepOverCheck = MAX_ROT;
  bool stopSensorTop = true;
  bool stopSensorBottom = true;

  do {
    motorStep(false, 1); // Rotate CCW until right sensor is interrupted
    stepOverCheck--;

    if (stepOverCheck < 0) {
      Serial.print("ST,0,INIT,ERR-MAX_ROT,ED\r\n");
      return;
    }
////"ST,0,MOTOR,0,30,90,ED\r\n"

    stopSensorTop = digitalRead(TOP_SENSOR_PIN) == LOW;
    stopSensorBottom = digitalRead(BOTTOM_SENSOR_PIN) == LOW;

    if (stopSensorBottom) {
      delay(_step_delay);
      stopMotor();
     motorStep(true, SENSOR_BACK_STEP);
      delay(_step_delay);
      stopMotor();
      break;
    }
  } while (stopSensorTop == false && stopSensorBottom == false);

  //_step_start_position = 0;
  _step_current_position = 0;
}
/*
void commandProcess(String commandStr) {
  commandStr.trim();

  String cmd = inputValues(commandStr, ',', 2);

  if (cmd.equals("CONN")) {
    if (Serial.available() > 0) {
      Serial.print("ST,0,RETCONN,1,ED\r\n"); // Connected
    } else {
      Serial.print("ST,0,RETCONN,0,ED\r\n"); // Not Connected
    }
    return;
  }

  if (cmd.equals("MOTOR")) {
    motorControl(commandStr);
  } else if (cmd.equals("DISPENSER") || cmd.equals("CARD_READER") || cmd.equals("PASSPORT_READER") || cmd.equals("PRINTER")) {
    relayControl(commandStr);
  } else {
    Serial.print("ST,0," + cmd + ",ERR_INVALID_CMD,ED\r\n");
  }
}
*/

void commandProcess(String commandStr) {
  commandStr.trim();

  String cmd = inputValues(commandStr, ',', 2);

  if (cmd.equals("MOTOR")) {
    motorControl(commandStr);
  //if (cmd.equals("HOME")) {
    //homePosition();
  //} 
  } else if (cmd.equals("DISPENSER") || cmd.equals("CARD_READER") || cmd.equals("PASSPORT_READER") || cmd.equals("PRINTER")) {
    relayControl(commandStr);
  } else {
    Serial.print("ST,0," + cmd + ",ERR_INVALID_CMD,ED\r\n");
  }
}


void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readString();
    commandProcess(data);
  }
}

//ST,0,MOTOR,0,60,80,ED\r\n

//"ST,0,MOTOR,0,30,90,ED\r\n"
//"ST,0,CARD_READER,1,ED\r\n"
//"ST,0,PRINTER,1,ED\r\n"
//"ST,0,PRINTER,0,ED\r\n"
//"ST,0,PASSPORT_READER,1,ED\r\n"
