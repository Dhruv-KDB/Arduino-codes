int gas_sensor = A0; //Sensor pin
float R0 = 10; //Sensor Resistance in fresh air from previous code
float b=1.133;
float m=-0.318;
void setup() {
  Serial.begin(9600); //Baud rate

  pinMode(gas_sensor, INPUT); //Set gas sensor as input
}


void loop() {
  float sensor_volt; //Define variable for sensor voltage
  float RS_gas; //Define variable for sensor resistance
  float RS_air; //Define variable fro sensor resistnace in air
  float R0; //Define variable for R0
  float sensorValue; //Define variable for analog readings
  
  for (int x = 0 ; x < 500 ; x++){
    sensorValue = sensorValue + analogRead(A0); //Add analog values of sensor 500 times
  }
  sensorValue = sensorValue / 500.0; //Take average of readings
  sensor_volt = sensorValue * (5.0 / 1023.0); //Convert average to voltage
  RS_air = ((5.0 * 10.0) / sensor_volt) - 10.0; //Calculate RS in air
  R0 = RS_air / 4.4; //Calculate R0
  

  
  float ratio; //Define variable for ratio
  sensor_volt = sensorValue * (5.0 / 1023.0); //Convert average to voltage
  RS_gas = ((5.0 * 10.0) / sensor_volt) - 10.0; //Calculate RS for gas
  ratio = RS_gas / R0;  

  double ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
  double ppm = pow(10, ppm_log);
  Serial.print("R0 = ");
  Serial.println(R0);//Convert ppm value to log scale
  Serial.print("PPM methane = ");
  Serial.println(ppm);
  delay(2000);
}
