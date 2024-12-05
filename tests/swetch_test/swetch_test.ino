#include <L298N.h>
#include <QTRSensors.h>
#define ll long long
#define ld long double

void motor_drive(int left, int right);
void calibrate();
void swetch();
void bangbangw();

const ll EN_A = 9;
const ll IN1_A = 11;
const ll IN2_A = 10;

const ll IN1_B = 6;
const ll IN2_B = 3;
const ll EN_B = 5;


int L=A1;
int ML=A2;
int MM=A3;
int MR=A4;
int R=A5;
int VR=8;
int VL=4;


const int arr[6] = {A0,L,ML,MM,MR,R};

L298N motorLB(EN_A, IN1_A , IN2_A);
L298N motorRA(EN_B, IN1_B , IN2_B);
////////////////////////////////////////////////////
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
////////////////////////////////////////////////////

ld lastError = 0.0;
const ll GOAL   = 2000;
const ll MAX_SPEED = 90;
ll position;
ll error;
ll adjustment;
const ld  KP  = 0.095;  
const ld KD  = 0;

///////////////////

ld lastErrorw = 0.0;
ll positionW;
ll errorw;
ll adjustmentw;

///////////////////

unsigned ll grid_time    = 0 ;
unsigned ll grid_counter = 0 ;
unsigned ll flag         = 0 ; 

////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4, A5}, SensorCount);
////////////////////////////////////////////////////
  pinMode(VL,INPUT);
  pinMode(L,INPUT);
  pinMode(ML,INPUT);
  pinMode(MM,INPUT);
  pinMode(MR,INPUT);
  pinMode(R,INPUT);
  pinMode(VR,INPUT);
////////////////////////////////////////////////////
  pinMode(EN_A,OUTPUT); 
  pinMode(EN_B,OUTPUT); 
  pinMode(IN1_A,OUTPUT);
  pinMode(IN2_A,OUTPUT);
  pinMode(IN1_B,OUTPUT);
  pinMode(IN2_B,OUTPUT);
////////////////////////////////////////////////////
  calibrate();
}



void loop()
{
//////////////////////  
swetch();
////////////////////// 
}

///////////////////////////////////////////////////////////////////////////////////////
void PIDs(){

  position = qtr.readLineBlack(sensorValues);

  error = GOAL - position;

  adjustment = KP*error + KD*(error - lastError);

  lastError = error;

  motor_drive( (MAX_SPEED - adjustment)  ,  (MAX_SPEED + adjustment) );

}
///////////////////////////////////////////////////////////////////////////////////////
void motor_drive(int left1, int right1){
  
  int left = constrain(left1, 0, MAX_SPEED);
  int right = constrain(right1, 0, MAX_SPEED);

  if(right > 0)
  {
    motorRA.setSpeed(right);
    motorRA.forward();
  }
  else if(right < 0)
  {
    motorRA.setSpeed(right);
    motorRA.backward();
  }
  else 
  {
    motorRA.setSpeed(0);
    motorRA.stop();
  }
  
 
  if(left > 0)
  {
    motorLB.setSpeed(left);
    motorLB.forward();
  }
  else if(left < 0)
  {
    motorLB.setSpeed(left);
    motorLB.backward();
  }
  else
  {
    motorLB.setSpeed(0);
    motorLB.stop();
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void calibrate(){

  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

}

///////////////////////////////////////////////////////////////////////////////////////  

void swetch(){

  while(

      ((digitalRead(VR)==0) and (digitalRead(VL)==0)) and //  <----     problem in normal turns
      (
      (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 150) or
      (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 150) or
      (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 150) or
      (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 150) or
      (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 150) 
      )
    )
    {
    
      Serial.println("swetch fn");
      //bangbangw();
    }

}
///////////////////////////////////////////////////////////////////////////////////////
void bangbangw(){
  if((sensorValues[1] <= qtr.calibrationOn.minimum[1] + 150) or (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 150)){
    motor_drive(MAX_SPEED,MAX_SPEED-50);
  }
  else if((sensorValues[3] <= qtr.calibrationOn.minimum[3] + 150) or (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 150)){
    motor_drive(MAX_SPEED-50,MAX_SPEED);
  }
}