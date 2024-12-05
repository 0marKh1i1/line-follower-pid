void setup() {
Serial.begin(9600);
  
pinMode(12,INPUT_PULLUP);
pinMode(13,INPUT_PULLUP);
}

void loop() {
  
  
if (digitalRead(12)==0){

  Serial.println("Button black");
  delay(1000);

}

if (digitalRead(7)==0){

  Serial.println("Button red");
  delay(1000);

}
/////////////////////////////////////////////

  

  Serial.print("\t");
  Serial.print(analogRead(A1));
  Serial.print("\t");
  Serial.print(analogRead(A2));
  Serial.print("\t");
  Serial.print(analogRead(A3));
  Serial.print("\t");
  Serial.print(analogRead(A4));
  Serial.print("\t");
  Serial.print(analogRead(A5));
  Serial.print("\t");
  Serial.print(digitalRead(8));
  Serial.print("\t");
  Serial.println(digitalRead(13));
  delay(50);

//////////////////////////////////////////////////



}
















