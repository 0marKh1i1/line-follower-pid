//in grid function test the time until all sensors become white then set it above
//test the pid and speed values

#include <L298N.h>
#include <QTRSensors.h>

void motor_drive(int left, int right);
void search_for_black();
void all_white();
void calibrate();
void start();
void PIDs();
void grid();

const unsigned int EN_A   = 9;
const unsigned int IN1_A  = 11;
const unsigned int IN2_A  = 10;

const unsigned int IN1_B  = 6;
const unsigned int IN2_B  = 3;
const unsigned int EN_B   = 5;


const int L   = A1;
const int ML  = A2;
const int MM  = A3;
const int MR  = A4;
const int R   = A5;
const int VR  = 8;
const int VL  = 4;

unsigned long long  grid_time    = 0 ;
int                 grid_counter = 0 ;
unsigned int        flag         = 0 ; 

const int red = 7; // red boutton

L298N motorLB(EN_A, IN1_A , IN2_A);
L298N motorRA(EN_B, IN1_B , IN2_B);
////////////////////////////////////////////////////
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
//////////////////////////////////////////////////// 4.06

double              lastError   = 0.0;
const int           GOAL        = 2000;
unsigned int        MAX_SPEED;
const unsigned int  WMAX_SPEED  = 72;//95
unsigned int  BMAX_SPEED  = 81;//105 
int                 position;
int                 error;
int                 adjustment;
const double        KP          = 0.15;  // 0.15  , 1
const double        KD          = 0.1;  //  0.1   , 0.075

int                 LiftOrRight  = 0; // b7bk
unsigned long long  ammar_time   = 0;
int                 first_time   = 1;
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
  pinMode(red,INPUT_PULLUP);
  ////////////////////// make it for 5 sec or more
  calibrate();
  //////////////////////
  while(digitalRead(red)==0){}
  start();
}

////////////////////////////////////////////////////

void start(){

  delay(3000);
  motorRA.setSpeed(84); motorLB.setSpeed(80);
  motorRA.forward();           motorLB.forward();
  delay(300); 
  ammar_time = millis();

}
////////////////////////////////////////////////////


void loop()
{
  if(
      (millis() - ammar_time) <= 15000 ) BMAX_SPEED=75;
      else BMAX_SPEED=80;

/*   if(first_time == 1){
    start();
    first_time = 2;
  } */
////////////////////// 
qtr.readCalibrated(sensorValues);
////////////////////// 
all_white();
//////////////////////
PIDs();
////////////////////// 
grid();
//////////////////////
delay(10);
//////////////////////
}
///////////////////////////////////////////////////////////////////////////////////////
void PIDs(){
     
    position = qtr.readLineBlack(sensorValues);
    MAX_SPEED = BMAX_SPEED;
    
    qtr.readCalibrated(sensorValues);

    if (((digitalRead(VR) == 0) && (digitalRead(VL) == 0)) && (
        (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) ||
        (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) ||
        (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) ||
        (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) ||
        (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)))
    {
        position = qtr.readLineWhite(sensorValues);
        MAX_SPEED = WMAX_SPEED;
       
    }
    else if (
      ((millis() - ammar_time) > 15000 ) &&
      (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) && 
      (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) && 
      (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) &&
      (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) &&
      (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)    
    ){
        search_for_black();

    }else if(
      ((millis() - ammar_time) <= 15000 ) &&
      (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) && 
      (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) && 
      (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) &&
      (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) &&
      (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)  
    ){
        motor_drive(84,80);// delete 
        delay(80);        // maybe delay is enough or not ?
        qtr.readCalibrated(sensorValues);
        
  if((sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) && 
      (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) && 
      (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) &&
      (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) &&
      (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250))
      search_for_black();

    }
    

  error = GOAL - position;

  adjustment = KP*error + KD*(error - lastError);

  lastError = error;

  motor_drive( (MAX_SPEED - adjustment)  ,  (MAX_SPEED + adjustment) );

}
///////////////////////////////////////////////////////////////////////////////////////
void motor_drive(int left1, int right1){
  
  int left = constrain(left1, 0, MAX_SPEED+10);
  int right = constrain(right1, 0, MAX_SPEED+10);


  if(right > 0)
  {
    motorRA.setSpeed(right);
    motorRA.forward();
  }
  else if(right < 0)
  {
    motorRA.setSpeed(abs(right));
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
    motorLB.setSpeed(abs(left));
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
// git it back to 5 sec i < 200; 
  for (uint16_t i = 0; i < 200; i++)// 2.5 sec
  {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

}
/////////////////////////////////////////////////////////////////////////////////////////
void grid(){
  
  if(
    (flag == 0 ) && 
    (sensorValues[0] >= qtr.calibrationOn.maximum[0] - 250) &&
    (sensorValues[1] >= qtr.calibrationOn.maximum[1] - 250) &&
    (sensorValues[2] >= qtr.calibrationOn.maximum[2] - 250) &&
    (sensorValues[3] >= qtr.calibrationOn.maximum[3] - 250) &&
    (sensorValues[4] >= qtr.calibrationOn.maximum[4] - 250)

  ){    
    flag = 1;

    grid_counter++;
    grid_time = millis();
    delay(100);
    qtr.readCalibrated(sensorValues);
  }


  if(grid_counter >= 3){
      motor_drive(92,90);
      delay(100);
      motor_drive(0,0);
      delay(100);
      qtr.readCalibrated(sensorValues);

    while(digitalRead(VR) == 1){
      motor_drive(80,0);
    }
    motor_drive(0,0);
    delay(100);
    qtr.readCalibrated(sensorValues);
    while(
      (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) && 
      (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) && 
      (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) 
    ){
        motor_drive(80,0);
        qtr.readCalibrated(sensorValues);
    }


    motor_drive(0,0);
    delay(100);
    qtr.readCalibrated(sensorValues);

    flag = 0;
    grid_counter = 0;

  }

  if (millis() - grid_time >= 1500 ){
      grid_counter = 0;
    }

  if( 
    (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) || 
    (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) || 
    (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) ||
    (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) ||
    (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)
  ) flag=0;
}
///////////////////////////////////////////////////////////////////////////////////////
void all_white(){

if(digitalRead(VL) == 0)//and digitalRead(VR) != 0
  LiftOrRight = 1; // sensor 0 and senso 1 are on left so you need to turn ledt so uoy turn on the right motor
else if(digitalRead(VR) == 0)
  LiftOrRight = 2;


}

///////////////////////////////////////////////////////////////////////////////////////  
void search_for_black(){
  unsigned long long last_time = millis(); 

if(LiftOrRight == 1){
  while(
        (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) and 
        (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) and 
        (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) and
        (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) and
        (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)    
        ){
          if(millis() - last_time > 30){
          motorRA.setSpeed(MAX_SPEED-5); motorLB.setSpeed(0);
          motorRA.forward();       motorLB.stop();
          }else{
            motorRA.setSpeed(MAX_SPEED); motorLB.setSpeed(MAX_SPEED);
            motorRA.forward();       motorLB.forward();
          }

          qtr.readCalibrated(sensorValues);

  }
  motor_drive(0,0);
  
} else if (LiftOrRight == 2){
  while(
        (sensorValues[0] <= qtr.calibrationOn.minimum[0] + 250) and 
        (sensorValues[1] <= qtr.calibrationOn.minimum[1] + 250) and 
        (sensorValues[2] <= qtr.calibrationOn.minimum[2] + 250) and
        (sensorValues[3] <= qtr.calibrationOn.minimum[3] + 250) and
        (sensorValues[4] <= qtr.calibrationOn.minimum[4] + 250)    
        ){
         if(millis() - last_time > 30){
          motorRA.setSpeed(0); motorLB.setSpeed(MAX_SPEED-5);
          motorRA.stop();       motorLB.forward();
         }else{
            motorRA.setSpeed(MAX_SPEED); motorLB.setSpeed(MAX_SPEED);
            motorRA.forward();       motorLB.forward();
          }

          qtr.readCalibrated(sensorValues);


  }
  
  motor_drive(0,0);

}else{
    motorRA.setSpeed(MAX_SPEED); motorLB.setSpeed(MAX_SPEED);
    motorRA.forward();           motorLB.forward();
  }


}




///////////////////////////////////////////////////////////////////////////////////////
