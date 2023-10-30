#include <QTRSensors.h>
#define left_motor_positive 2
#define left_motor_negative 3
#define right_motor_positive 5
#define right_motor_negative 4
#define en1 10
#define en2 11

int check = 0;
int attempt = 0;
int rightState =0;

int w = 550;
int b = 700;
int c = 500;

#define led 13


QTRSensors qtra;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];





int initial_motor_speed = 100;//100
int rotating_speed = 89;//60
int forward_speed = 150;//100
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;


int error; 
float kp = 1; //proportional constant 0.05
float ki = 0;
float kd = 7; //5
float P,I,D,previousError=0;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;

int ObstaclePin=10;
int ObstacleRead;
void led_signal(int times);

void calculatePID();
void PIDmotor_control();
   uint16_t position;

void readIRvalue();     //to read sensor value and calculate error as well mode
// void Set_motion();

void dryrun();
// void actualrun();

void recIntersection(char);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
// void setmotionactual();
// void mazeTurn (char dir);

void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void goAndTurnLeft();
void maze_end();
void move_inch();
void backward(int spd1, int spd2);


void setup() 
{ 
//  Serial.begin(9600);
  // put your setup code here, to run once:
  for (int i = 2; i <= 5; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
  delay(500);
  pinMode(13, OUTPUT);

  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, SensorCount);
 // qtra.setEmitterPin(9);

  
  digitalWrite(13, HIGH);  

  
 // turn on Arduino's LED to indicate we are in calibration mode
  left(rotating_speed);
  for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
  { 
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
stop_motor();
  delay(3000);
}


void loop() 
{   
    if(Status==0){
  dryrun();
}
    else{

      actualrun();
    } 
  }


void led_signal(int times)
{
  for (int i = 0; i <= times; i=i+1)
  {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }
}

//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
     readIRvalue();
     set_motion();
  
}
//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalue()
{
    uint16_t position = qtra.readLineWhite(sensorValues);
    
    error = 2500 - position;
      for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print("  ");
  }
  Serial.println(position);

  // mode = 'F';

    

//--------------------------------------------------------------------------------------
  if (sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b)
  {
    mode = 'N';  //NO LINE
    Serial.println("b");
    error = 0;
  }

  //--------------------------------------------------------------------------------------
  else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
   else if ( sensorValues[0] < w &&  sensorValues[2] < w && sensorValues[3] < w &&  sensorValues[5] < w)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
  //--------------------------------------------------------------------------------------

  else if (sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w )
  {
    mode = 'R';//90 degree turn
    error = 0;
    Serial.println("R");
  }
  //   else if (sensorValues[0] > b && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w )
  // {
  //   mode = 'R';//90 degree turn
  //   error = 0;
  //   Serial.println("R");
  // }
  // else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[4] < w && sensorValues[5] < w)
  // {
  //   mode = 'R';//90 degree turn
  //   Serial.println("R");
  //   error = 0;
  // }


  

  else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] > b && sensorValues[5] > b)
  {
    mode = 'L'; //90 degree turn
    Serial.println("l");
    error = 0;
  }
  // else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[4] > b && sensorValues[5] > b)
  // {
  //   mode = 'L'; //90 degree turn
  //   error = 0;
  //   Serial.println("l");
  // }
  else{
    mode = 'F';
  }
  

}



//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion()
{
  switch (mode)
  {
    case 'N':
      //stop_motor();
      move_inch();
      goAndTurnRight();
      recIntersection('B');
      break;
    case 'S':
      move_inch();
move_inch();
      move_inch();
      move_inch();
            readIRvalue();
      
      
      if (mode == 'S')
      {
        maze_end();
      }
      else
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case 'R':
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();
            readIRvalue();
      
      if (mode == 'F')
     {
        recIntersection('S');
      }
     else
      {
          goAndTurnRight();
      }
      break;
    case 'L':
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();
       readIRvalue();
          if (mode == 'F')
     {
        recIntersection('L');
           goAndTurnLeft();
      }
     else
      {
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
void move_inch()
{
  forward(forward_speed, forward_speed);
  delay(100);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}
//-----------------------------------------------------------------------------------------------------------
void goAndTurnLeft()
{ 
    previousError = 0;
  left(rotating_speed);
  delay(600);
  do
  {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F' );
     left(rotating_speed);
  delay(75);
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight()
{previousError = 0;
  right(rotating_speed);
  delay(600);
  do
  {
    right(rotating_speed);
    readIRvalue();
  }while (mode != 'F' );
  right(rotating_speed);
  delay(75);
 
}
//-----------------------------------------------------------------------------------------------
void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
  delay(5000);
}
//------------------------------------------------------------------------------------------------
void calculatePID()
{
  
  P = error;
  I = I + error;
  D = error-previousError;
  pid_value = (kp*P) + (ki*I) + (kd*D);
  previousError = error;
  
}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);
  
}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}

void backward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
}

void left(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, HIGH);
}

void right(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
}
//--------------------------------------------------------------------------------------


void actualrun(void)
{if(Status<2000){
  Status++;
}
  while (Status ==2000)
  {
    readIRvalue();
    setmotionactual();
  }
}

//actual runfunctions--------------------------------------------------------------------------------------------
void setmotionactual()
{
  switch (mode)
  {
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
    case 'N':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {

      
        move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      readIRvalue();
      if (mode == 'S')
      {
        maze_end();
      }
      else
      {
      mazeTurn (path[pathIndex]);
        pathIndex++;
      }
       
      // }
      break;
    case 'L':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {
             move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      readIRvalue();
      if (mode == 'F')
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      else
      {
      goAndTurnLeft();
      }
  
      // }
      break;
    case 'R':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {
                 move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      readIRvalue();
      if (mode == 'F')
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      else
      {
      goAndTurnRight();
      }
      // }
      break;
  }

}

void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  
  simplifyPath(); 
  pathLength ++;// Simplify the learned path.
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path[pathLength - 2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for (i = pathLength - 1; i > 0 && i > pathLength - 4; i--)
  {
    switch (path[i])
    {
      case 'R':
        totalAngle += 270;
        break;
      case 'L':
        totalAngle += 90;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle)
  {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'L';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'R';
      break;
  }
  // The path is now two steps shorter.
  pathLength -= 2;
}

void mazeTurn (char dir)
{
  switch (dir)
  {
    case 'L': // Turn Left
      goAndTurnLeft();
      break;

    case 'R': // Turn Right
      goAndTurnRight();
      break;

    case 'B': // Turn Back
      goAndTurnLeft();
      break;

    case 'S': // Go Straight
      move_inch();
      break;
  }
}
