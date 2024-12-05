#include <QTRSensors.h>
//functions

void calibrate();
void print_pos();

// sensors pins
int L=A1;
int ML=A2;
int MM=A3;
int MR=A4;
int R=A5;
//sensor array
const int arr[6] = {A0,L,ML,MM,MR,R};
int threshold  [5] = {500,500,500,500,500};

////////////////////////////////////////////////////
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
////////////////////////////////////////////////////


int position;

////////////////////////////////////////////////////
// 0 255 MAX_SPEED/2000 
void setup()

{
 // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial.println("started");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4, A5}, SensorCount);

////////////////////////////////////////////////////

  pinMode(L,INPUT);
  pinMode(ML,INPUT);
  pinMode(MM,INPUT);
  pinMode(MR,INPUT);
  pinMode(R,INPUT);

////////////////////////////////////////////////////

calibrate();

}

void loop()
{
print_pos();
}

///////////////////////////////////////////////////////////////////////////////////////








///////////////////////////////////////////////////////////////////////////////////////
void print_pos(){
  position =  qtr.readLineBlack(sensorValues);
  qtr.readCalibrated(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}
///////////////////////////////////////////////////////////////////////////////////////
void calibrate(){

delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
    Serial.println(' ');

    for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(' ');

  }
  Serial.println();
  
  Serial.println();
  delay(1000);




}


