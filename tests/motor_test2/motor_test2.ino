#include <L298N.h>

//motor pins
const unsigned int EN_A = 9;
const unsigned int IN1_A = 11;
const unsigned int IN2_A = 10;

const unsigned int IN1_B = 6;
const unsigned int IN2_B = 3;
const unsigned int EN_B = 5;

int speed =100;

void motor_drive(int left, int right);
void test();

// Initialize both motors
L298N motorLB(EN_A, IN1_A , IN2_A);
L298N motorRA(EN_B, IN1_B , IN2_B);

void setup() {
Serial.begin(9600);
}

void loop() {


test();
delay(1000);

/////////////////////////////////

}

void motor_drive(int left1, int right1){
  
  int left = constrain(left1, 0, 110);
  int right = constrain(right1, 0, 110);

if(right>7)right-=7;/// <<<<<------ hardware issue
  
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


void test(){
motor_drive(0,speed);
Serial.println("Right");
delay(3000);

motor_drive(speed, 0);
Serial.println("Left");
delay(3000);

motor_drive(speed, speed);
Serial.println("forward");
delay(5000);

motor_drive(-speed, -speed);
Serial.println("backward");
delay(3000);

motor_drive(0, 0);
Serial.println("stop");
delay(1000);
}



