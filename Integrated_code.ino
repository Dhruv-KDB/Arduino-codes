#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C Interface
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

const int analogInPin = A0; 
int sensorValue = 0; 
unsigned long int avgValue; 
float b;
int buf[10],temp;


int moisture_sensor_pin = A1; 
int output_value ;

int gas_sensor = A2; //Sensor pin
float R0 = 10; //Sensor Resistance in fresh air from previous code
float b=1.133;
float m=-0.318;

int uv_sensorPin = A3;  // input pin 
int UV_val = 0;// variable to store the value coming from the sensor
int UVIndex = 0;



void ph_sensor();
void temp_sensor();
void moisture_sensor();
void methane_sensor();
void uv_sensor();
void pressure_sensor();

void setup()
{
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  sensors.begin();  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
}
void loop()
{
  void ph_sensor();
  delay(1000);

  void temp_sensor();
  delay(1000);

  void moisture_sensor();
  delay(1000);

  void methane_sensor();
  delay(1000);

  void uv_sensor();
  delay(1000);

  void pressure_sensor();
  delay(1000);
}

void ph_sensor()
{
  for(int i=0;i<10;i++) 
 { 
  buf[i]=analogRead(analogInPin);
  delay(10);
 }
 for(int i=0;i<9;i++)
 {
  for(int j=i+1;j<10;j++)
  {
   if(buf[i]>buf[j])
   {
    temp=buf[i];
    buf[i]=buf[j];
    buf[j]=temp;
   }
  }
 }
 avgValue=0;
 for(int i=2;i<8;i++)
 avgValue+=buf[i];
 float pHVol=(float)avgValue*5.0/1024/6;
 float phValue = -5.70 * pHVol + 21.34;
 Serial.print("sensor = ");
 Serial.println(phValue);
  delay(20);
}

void temp_sensor()
{
    // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperature is: ");
 
    delay(1000);
}

void moisture_sensor()
{
   output_value= analogRead(moisture_sensor_pin);
  output_value = map(output_value,550,0,0,100);
  Serial.print("Mositure : ");
  Serial.print(output_value);
  Serial.println("%");
  delay(1000);
}

void methane_sensor()
{
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

void uv_sensor()
{
   UV_val = analogRead(uv_sensorPin);// read analog value between 0 to 1023
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

void pressure_sensor()
{
   Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()); //displaying the Pressure in Pascals
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day
    Serial.println(" m");                    

    Serial.println();
    delay(2000);
}
  
