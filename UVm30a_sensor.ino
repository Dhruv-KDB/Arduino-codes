int sensorPin = A0;  // input pin 
int UV_val = 0;// variable to store the value coming from the sensor
int UVIndex = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
}

void loop() {
  UV_val = analogRead(sensorPin);// read analog value between 0 to 1023
  Serial.print("UV_val ");
  Serial.println(UV_val); 
   int voltage = (UV_val * (5.0 / 1023.0))*1000;  //Voltage in miliVolts
   Serial.print("voltage= ");
   Serial.println(voltage);
//print digital value on serial monitor
if(voltage<=227)
  {
    UVIndex = "0";
  
  }else if (voltage>227 && voltage<=318)
  {
    UVIndex = "1";
  }
  else if (voltage>318 && voltage<=408)
  {
    UVIndex = "2";
  }else if (voltage>408 && voltage<=503)
  {
    UVIndex = "3";
  }
  else if (voltage>503 && voltage<=606)
  {
    UVIndex = "4";
  }else if (voltage>606 && voltage<=696)
  {
    UVIndex = "5";
  }else if (voltage>696 && voltage<=795)
  {
    UVIndex = "6";
  }else if (voltage>795 && voltage<=881)
  {
    UVIndex = "7";
  }
  else if (voltage>881 && voltage<=976)
  {
    UVIndex = "8";
  }
  else if (voltage>976 && voltage<=1079)
  {
    UVIndex = "9";
  }
  else if (voltage>1079 && voltage<=1170)
  {
    UVIndex = "10";
  }else if (voltage>1170)
  {
    UVIndex = "11";
  }
  //delay (1000);
  //return UVIndex;
Serial.print("UV_INDEX = ");
Serial.println(UVIndex);
  delay(1000);
}
