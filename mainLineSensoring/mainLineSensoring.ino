#include <QTRSensors.h>
#define en1 10
#define en2 11
#define buttom_reading 8


int w = 350;
int b = 600;


#define led 13


QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];




int initial_motor_speed = 200;  //100
int rotating_speed = 140;       //60
int forward_speed = 220;        //100
int right_motor_speed = 0;      //for the speed after PID control
int left_motor_speed = 0;


int error;
float kp = 2;  //proportional constant 0.05
float ki = 0;
float kd = 7;  //5
float P, I, D, previousError = 0;
int pid_value;

char mode;
int Status = 0;


void led_signal(int times);

void calculatePID();
void PIDmotor_control();
uint16_t position;
inline void readIRvalue() __attribute__((always_inline));
inline void Set_motion() __attribute__((always_inline));


inline void recIntersection(char) __attribute__((always_incline));
char path[100] = "";
unsigned char pathLength = 0;  // the length of the path
int pathIndex = 0;
// void setmotionactual();
// void mazeTurn (char dir);

inline void forward(int spd1, int spd2) __attribute__((always_inline));
;
inline void left(int spd) __attribute__((always_inline));
;
inline void right(int spd) __attribute__((always_inline));
;
inline void stop_motor() __attribute__((always_inline));
;
inline void goAndTurnLeft() __attribute__((always_inline));
;
inline void maze_end() __attribute__((always_inline));
;
inline void move_inch() __attribute__((always_inline));
;
inline void backward(int spd1, int spd2) __attribute__((always_inline));
;


void setup()



{
  Serial.begin(9600);
  // put your setup code here, to run once:

  DDRB = B0011100;
  DDRD = B0101100;
  DDRD &= !B0100001;

  PORTB = B0000000;

  delay(500);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);

  PORTD |= B0100000;  // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  PORTD &= !B0100000;
  delay(1000);
  while (true) {
    if (digitalRead(buttom_reading) == HIGH) break;
  }
  delay(1000);
}


void loop() {



  //READ VALUE


  readIRvalue();

  set_motion();

  //   unsigned long startTime = millis();

  //   // Code to be measured
  //  dryrun();

  //   unsigned long endTime = millis();
  //   unsigned long elapsedTime = endTime - startTime;

  //   Serial.print("Execution Time: ");
  //   Serial.print(elapsedTime);
  //   Serial.println(" ms");

  //   delay(1000);  // Adjust as needed


  //    if(Status==0){
  //   dryrun();
  // }
  //     else{
  //       actualrun();
  //     }
  //   }
}

void led_signal(int times) {
  for (int i = 0; i <= times; i = i + 1) {
    PORTD |= B0100000;
    delay(100);
    PORTD &= !B0100000;
    delay(100);
  }
}

void readIRvalue() {

  uint16_t position = qtr.readLineBlack(sensorValues);

  error = 2500 - position;


  //   for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  // mode = 'F';



  //--------------------------------------------------------------------------------------
  if (sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b) {
    mode = 'N';  //NO LINE
    error = 0;
  }

  //--------------------------------------------------------------------------------------
  else if (sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w) {
    mode = 'S';  //Stop Condition
    error = 0;
  }

  //--------------------------------------------------------------------------------------

  else if (sensorValues[0] > b && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w) {
    mode = 'R';  //90 degree turn
    error = 0;
  }

  else if (sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[5] > b) {
    mode = 'L';  //90 degree turn
    error = 0;
  }

  else {
    mode = 'F';
  }
}


//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion() {
  switch (mode) {
    case 'N':
      move_inch();
      goAndTurnRight();
      recIntersection('B');
      break;
    case 'S':
      move_inch();
      readIRvalue();

      if (mode == 'S') {
        //  maze_end();
      } else {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case 'R':

      move_inch();
      readIRvalue();

      if (mode == 'F') {
        recIntersection('S');
      } else {
        goAndTurnRight();
      }
      break;
    case 'L':
      move_inch();
      readIRvalue();

      if (mode == 'F') {
        goAndTurnLeft();
        recIntersection('L');
      } else {
        goAndTurnLeft();
      }
      break;
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
  }
}

//-----------------------------------------------------------------------------------------------------------
void move_inch() {
  forward(forward_speed, forward_speed);
  delay(69);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor() {
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  PORTB &= !B00111100;

  // digitalWrite(led, LOW);
}
//-----------------------------------------------------------------------------------------------------------
void goAndTurnLeft() {
  previousError = 0;

  do {
    left(rotating_speed);
    readIRvalue();
  } while (mode == 'F');
  left(rotating_speed);
  delay(50);
  do {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F');

  do {
    left(rotating_speed);
    readIRvalue();
  } while (position < 2000);
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight() {
  previousError = 0;
  do {
    right(rotating_speed);
    readIRvalue();
  } while (mode == 'F');

  right(rotating_speed);
  delay(300);
  do {
    right(rotating_speed);
    readIRvalue();
  } while (mode != 'F');

  do {
    right(rotating_speed);
    readIRvalue();
  } while (position > 3000);
}
//-----------------------------------------------------------------------------------------------
void maze_end() {
  Status++;
  stop_motor();
  led_signal(20);
  delay(10000);
}
//------------------------------------------------------------------------------------------------
void calculatePID() {

  P = error;
  I = I + error;
  D = error - previousError;
  pid_value = (kp * P) + (ki * I) + (kd * D);
  previousError = error;
}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control() {
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);
}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2) {
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  PORTB |= B00101000;
  PORTB &= !B00010100;
  digitalWrite(led, LOW);
}

void backward(int spd1, int spd2) {
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  PORTB |= B00010100;
  PORTB &= !B00101000;
  digitalWrite(led, HIGH);
}

void left(int spd) {
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  PORTB |= B00011000;
  PORTB &= !B00100100;
  digitalWrite(led, HIGH);
}

void right(int spd) {
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  PORTB |= B00100100;
  PORTB &= !B00011000;
  digitalWrite(led, HIGH);
}
//--------------------------------------------------------------------------------------
void recIntersection(char Direction) {
  path[pathLength] = Direction;  // Store the intersection in the path variable.
  pathLength++;
  // simplifyPath(); // Simplify the learned path.
}
