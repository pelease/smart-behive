#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <FirebaseArduino.h>
#include <ESP8266HTTPClient.h>
#include "MegunoLink.h"
#include <Filter.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

///////////////////MQ-135///////////////////////////
#include <MQUnifiedsensor.h>

//Definitions
#define Board "ESP8266"
#define Voltage_Resolution 3.3
#define pin A0 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  

//Declare Sensor
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ru.pool.ntp.org", 3600, 60000);

ExponentialFilter<long> ADCFilter(5,0);

#define DHTPIN 4 
#define DHTTYPE DHT11
#define NOISE 550

DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "hse"; // wi-fi name
const char* password = "hsepassword"; // wi-fi password

#define FIREBASE_HOST "smart-beehive-64a42-default-rtdb.firebaseio.com" // firebase address
#define FIREBASE_AUTH "CoeMZ8khgYZ1VJUyoft77va97UtBYuUSz7CiMVrK" // firebase token

void setup() {
    Serial.begin(9600);
    dht.begin();
    Serial.println();
    Serial.print("Connecting to "); 
    Serial.print(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    } 
    Serial.println("");
    
    Serial.print("WiFi connected, IP address: "); 
    Serial.println(WiFi.localIP());

    Serial.println("NodeMCU as a Server Role Started");
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get NH4 concentration
    MQ135.init(); 

    /*
      Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937  
    Alcohol  | 77.255 | -3.18 
    CO2      | 110.47 | -2.862
    Tolueno  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Acetona  | 34.668 | -3.369
    */

   Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/1);
  Serial.println("  done!.");
  timeClient.begin();
}

void loop() {
    timeClient.update();
    time_t seconds = time(NULL);
    tm* timeinfo = localtime(&seconds);
    // MQ135 gasSensor = MQ135(A0);
    // float gas = analogRead(A0);
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    // float air_quality = gasSensor.getPPM();
    // float zero = gasSensor.getRZero();


    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    float gas = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
    // float gas = random(1500, 1700);

    // n = abs(1023 - n);
    // n = (n <= NOISE) ? 0 : abs(n - NOISE);
    // ADCFilter.Filter(n);
    // int lvl = ADCFilter.Current();


    int n = analogRead(A0);
    float fireSound = (n + 83.2073) / 11.003;

    String date = timeClient.getFormattedTime();

    if (isnan(h) || isnan(t))
    {                                   
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    } 
    Serial.print("-  Humidity: ");  
    Serial.print(h);
    Serial.println("%");

    Serial.print("-  Temperature: ");  
    Serial.print(t);  
    Serial.println("°C ");
    
    Serial.print("-  Air_quality: ");  
    Serial.print(gas);  
    Serial.println("PPM ");

    Serial.print("-  Noise: ");  
    Serial.print(fireSound);
    Serial.println("dBm ");

    Serial.println(date);

    delay(2000);


    Firebase.setFloat("Humidity", h);
    Firebase.setFloat("Temperature", t);
    Firebase.setFloat("Air_quality", gas);
    Firebase.setFloat("Noise", fireSound);
    Firebase.setString("Date", date);

    if (Firebase.failed()) 
    {
 
      Serial.print("pushing /logs failed:");
      Serial.println(Firebase.error()); 
      return;
  }
}