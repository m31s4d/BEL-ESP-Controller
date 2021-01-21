/**
 This code was written/hacked together by Swen Schreiter and is the basis for the multifunctional environmental sensing device called S.E.E.D (Small Electronic Environmental Device).
Project details can be found on GitHub (https://github.com/m31s4d/Project-S.E.E.D) or at the project blog (TBD).
 */

// Include the libraries we need
//#include <Arduino.h>
#include <Wire.h>    //Adds library for the I2C bus on D1 (SCL) and D2(SCD) (both can be changed on the ESP8266 to the deisred GPIO pins)
//#include <SPI.h>
#include <OneWire.h>           //Adds library needed to
#include <DallasTemperature.h> //Adds the Dallas Temp library to use DS18B20 with the Wemos
#include <Adafruit_BME280.h>   //Adds library for the BME280 (Bosch) environmental sensor
#include <Adafruit_Sensor.h>   //Adds library for all Adafruit sensors to make them usable
#include "ESP8266WiFi.h"       // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h"      // Allows us to connect to, and publish to the MQTT broker

//Initialization of the OneWire Bus und the temp sensor
const int oneWireBus = D3;                 // GPIO where the DS18B20 is connected to
int numDevices;                            // Number of temperature devices found
OneWire oneWire(oneWireBus);               // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature dallassensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress tempDeviceAddress;           // We'll use this variable to store a found device address

//Initialization of all environmental variables as global to share them between functions
float bme280_temp = 0;     //Sets the variable temp to the temp measure of the BME280
float bme280_humidity = 0; //Sets variable bme_humidity to humidity measure of BME280
float bme280_pressure = 0; //Sets variable bme_pressure to pressure measire of BME280
float bme280_altitude = 0; //Sets the altitutde variable bme_altitutde to zero
float sht1750_lux = 0;     //Sets the value lux to the measured value of the light sensor.

//Adafruit_BMP280 bmp; // use I2C interface
Adafruit_BME280 bme;                                     // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();  //Gets temperature value from the sensor
Adafruit_Sensor *bme_pressure = bme.getPressureSensor(); //Gets pressure value from sensor
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor(); //Gets humidity value from sensor/ initializes it

// MQTT 1
// These lines initialize the variables for PubSub to connect to the MQTT Broker 1 of the Aero-Table
const char *mqtt_server_1 = "192.168.2.105";                //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
const char *temp_bme280_topic_1 = "aeroponic/growtent1/temperatur/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_bme280_topic_2 = "aeroponic/growtent1/temperatur/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_1 = "aeroponic/growtent1/humidity/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_2 = "aeroponic/growtent1/humidity/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_1 = "aeroponic/growtent1/pressure/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_2 = "aeroponic/growtent1/pressure/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_ds18b20_topic_1 = "aeroponic/growtent1/temperatur/d18b20/sensor1"; //Adds MQTT topic for the dallas sens
const char *temp_ds18b20_topic_2 = "aeroponic/growtent1/temperatur/d18b20/sensor2"; //Adds MQTT topic for the dallas sens
const char *mqtt_connection_topic_1 = "aeroponic/growtent1/connection/sens<or1";
//const char* mqtt_username = "cdavid"; //if MQTT server needs credentials they need to be added in the next two lines
//const char* mqtt_password = "cdavid";
// MQTT 2
// These lines initialize the variables for PubSub to connect to the MQTT Broker 2 of the Aero-Table
const char *mqtt_server_2 = "XXX.XXX.XXX.XXX";          //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
const char *light_topic_2 = "home/bedroom/lightsensor"; //This defines a topic for our light measurment and where it should be published via MQTT (Channel)
const char *temp_topic_2 = "home/bedroom/tempsensor";   //This defines a topic for the temperature measurement of the BME280 at the MQTT broker
const char *mqtt_connection_topic_2 = "aero1/growtent/connection/sensor2";
//const char* mqtt_username = "cdavid"; //if MQTT server needs credentials they need to be added in the next two lines
//const char* mqtt_password = "cdavid";
const char *clientID = "ESP-Table_1-1"; // The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
unsigned long noLoop = 0; //Needed for the deepsleep loop function
unsigned long lastLoop1 = 0; //Needed for the millis() loop function
unsigned long lastLoop2 = 0; //Needed for the millis() loop function

WiFiClient wifiClient;                                // Initialise the WiFi and MQTT Client objects
PubSubClient client(mqtt_server_1, 1883, wifiClient); // 1883 is the listener port for the Broker //PubSubClient client(espClient);

void connect_wifi_1() {
  // WiFi connection settings
  const char *ssid = " ";                      //This line needs to be populated by the SSID of the wifi network we want to connect to
  const char *wifi_password = " "; //This line is populated by the password of the wifi network

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ClientID: ");
  Serial.println(clientID);
}
void connect_wifi_2() {
    //Defines the wifi connection settings of the second broker
  const char *ssid = "WLAN-173564";
  const char *wifi_password = "40568512259452661617";

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ClientID: ");
  Serial.println(clientID);
}
void connect_MQTT_1()
{ //Defines a function "connect_MQTT" which includes all necessary steps to connect the ESP with the server
  // Create a random client ID
  //String clientId = "SEED-";
  //clientId += String(random(0xffff), HEX);
  //PubSubClient client(espClient);
  //client.setServer(mqtt_server_1, 1883);  
  PubSubClient client(mqtt_server_1, 1883, wifiClient); // 1883 is the listener port for the Broker
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID))
  {
    Serial.println("Connected to MQTT Broker!");
    //MQTT Topics to subscribe to for functioning. Structure as follows: project/location/type/number
    //Following are test topics
    //client.subscribe(tempsensor1_topic);
    delay(100);
    //client.publish(mqtt_connection_topic_1, "on");
  }
  else
  {
    Serial.println("Connection to MQTT Broker failed...");
  }
}
void connect_MQTT_2()
{ 
   // Create a random client ID
  //String clientId = "SEED-";
  //String chipname = String(ESP.getChipId()); //Call to get ChipID to add to the client ID
  //clientId += chipname;
  //Serial.print(clientId);
  //clientId += String(random(0xffff), HEX);
  client.setServer(mqtt_server_2, 1883);  
  //Defines a function "connect_MQTT" which includes all necessary steps to connect the ESP with the server
  PubSubClient client(mqtt_server_2, 1883, wifiClient); // 1883 is the listener port for the Broker
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID))
  {
    Serial.println("Connected to MQTT Broker!");
    //MQTT Topics to subscribe to for functioning. Structure as follows: project/location/type/number
    //Following are test topics
    //client.subscribe(tempsensor1_topic);
    delay(100);
    client.publish(mqtt_connection_topic_2, "on");
  }
  else
  {
    Serial.println("Connection to MQTT Broker failed...");
  }
}
/*void measure_distance_ultrasonic(){
int trigger=D7;                        // Der Trigger Pin
int echo=D8;                                  // Der Echo Pin
float usonic_time=0;                                // Hier wird die Zeitdauer abgespeichert// die die Ultraschallwelle braucht// um zum Sensor zurückzukommen
float distance_measured=0;                           // Hier wird die Entfernung vom  // Hindernis abgespeichert 
pinMode(trigger, OUTPUT);
pinMode(echo, INPUT);
digitalWrite(trigger, LOW);              // Den Trigger auf LOW setzen um// ein rauschfreies Signal// senden zu können
delay(5);                                // 5 Millisekunden warten
digitalWrite(trigger, HIGH);             // Den Trigger auf HIGH setzen um eine // Ultraschallwelle zu senden
delay(10);                               // 10 Millisekunden warten
digitalWrite(trigger, LOW);              // Trigger auf LOW setzen um das  // Senden abzuschließen
usonic_time = pulseIn(echo, HIGH);             // Die Zeit messen bis die // Ultraschallwelle zurückkommt
distance_measured = ((usonic_time/2)/29.1); // 29.1;           // Die Zeit in den Weg in Zentimeter umrechnen
Serial.print(distance_measured);            // Den Weg in Zentimeter ausgeben
Serial.println(" cm");               //
delay(1000);      
}*/
void measure_temp()
{
  sensors_event_t temp_event; //, pressure_event, humidity_event; //Initialization of the sensor events to use them later
  bme_temp->getEvent(&temp_event);
  //bme_pressure->getEvent(&pressure_event);
  //bme_humidity->getEvent(&humidity_event);
  bme280_temp = bme.readTemperature(); //Sets the variable temp to the temp measure of the BME280
  //bme280_humidity = humidity_event.relative_humidity; //Sets variable bme_humidity to humidity measure of BME280
  //bme280_pressure = pressure_event.pressure; //Sets variable bme_pressure to pressure measire of BME280
  //float bme280_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  String temp = String((float)bme280_temp); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String humi=String((float)bme280_humidity); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String press=String((float)bme280_pressure); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String alti=String((float)bme280_altitude); //Important to change the value of the variable to a string
  //MQTT can only transmit strings
  if (client.publish(temp_bme280_topic_1, String(temp).c_str()) && client.publish(temp_bme280_topic_2, String(temp).c_str()))
  {                                  // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    Serial.print(temp + " sent to Table and Server!"); //To allow debugging a serial output is written if the measurment was published succesfully.
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else
  {
    Serial.println(temp);
    Serial.print(" failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(temp_bme280_topic_1, String(temp).c_str());
    client.publish(temp_bme280_topic_2, String(temp).c_str());
  }
}
void measure_humidity()
{
  sensors_event_t /*temp_event, pressure_event,*/ humidity_event; //Initialization of the sensor events to use them later
  bme_humidity->getEvent(&humidity_event);
  bme280_humidity = bme.readHumidity(); //Sets variable bme_humidity to humidity measure of BME280
  String humi = String((float)bme280_humidity);       //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //MQTT can only transmit strings
  if (client.publish(humidity_bme280_topic_1, String(humi).c_str()) && client.publish(humidity_bme280_topic_2, String(humi).c_str()))
  {                                  // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    Serial.println(humi + " sent to Table and Server!"); //To allow debugging a serial output is written if the measurment was published succesfully.
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else
  {
    Serial.println(humi);
    Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(humidity_bme280_topic_1, String(humi).c_str());
    client.publish(humidity_bme280_topic_2, String(humi).c_str());
  }
}
void measure_pressure()
{
  sensors_event_t /*temp_event,*/ pressure_event; /*, humidity_event*/ //Initialization of the sensor events to use them later
  //bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme280_pressure = bme.readPressure(); //Sets variable bme_pressure to pressure measire of BME280
  //bme280_pressure = pressure_event.pressure;     //Sets variable bme_pressure to pressure measire of BME280
  String press = String((float)bme280_pressure); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //MQTT can only transmit strings
  if (client.publish(pressure_bme280_topic_1, String(press).c_str()) && client.publish(pressure_bme280_topic_2, String(press).c_str()))
  {                                   // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    Serial.println(press + " sent!"); //To allow debugging a serial output is written if the measurment was published succesfully.
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else
  {
    Serial.println(press);
    Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(pressure_bme280_topic_1, String(press).c_str());
    client.publish(pressure_bme280_topic_2, String(press).c_str());
  }
}
/*void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String msg;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    msg = msg + (char)payload[i];
  }
  if (msg == "on")
  {
    Serial.println("I received an ON");
    u8x8.setFont(u8x8_font_chroma48medium8_r); //sets font for the oled display
    //u8x8.setPowerSave(0);
    u8x8.clearDisplay();
    u8x8.setCursor(0, 4); //This function allows us to put the cursor on the oled to a specified position (X, Y)
    u8x8.print("I received an ON");
  }
  else if (msg == "off")
  {
    Serial.println("I received an OFF");
    u8x8.setFont(u8x8_font_chroma48medium8_r); //sets font for the oled display
    //u8x8.setPowerSave(0);
    u8x8.clearDisplay();
    u8x8.setCursor(0, 4); //This function allows us to put the cursor on the oled to a specified position (X, Y)
    u8x8.print("I received an Off");
  }
}*/
void read_dallas()
{
  dallassensors.requestTemperatures(); //Sends a request to the DS18B20 to prepare a temperature reading
  // Loop through each device, print out temperature data
  for (int i = 0; i < numDevices; i++)
  {
    // Search the wire for address
    if (dallassensors.getAddress(tempDeviceAddress, i))
    {
      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.println(i, DEC);

      // Print the data
      float tempC = dallassensors.getTempC(tempDeviceAddress);
      String temp_dallas = String((float)tempC); //Important to change the value of the variable to a string
      Serial.print("Temp C: ");
      Serial.print(tempC);
      if (i==1)
      {                                         
        client.publish(temp_ds18b20_topic_1, String(temp_dallas).c_str()); // PUBLISH to the MQTT Broker (topic was defined at the beginning)
        Serial.println(temp_dallas + " sent!"); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else
      {
        Serial.println(temp_dallas);
        Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
        client.connect(clientID);
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(temp_ds18b20_topic_2, String(temp_dallas).c_str());
      }
    }
    //float temperatureC = dallassensors.getTempCByIndex(0); //Adds the value of the 0 device of temperature to the variable tempc
    //float temperatureF = sensors.getTempFByIndex(0);
    //Serial.print(temperatureC); //Prints the value of the variable tempc in the Serial Console
    //Serial.println("C");
    //Serial.print(temperatureF);
    //Serial.println("F");
    //delay(5000);
  }
}
void setup()
{
  Serial.begin(9600); // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();       // On esp8266 you can select SCL and SDA pins using Wire.begin(D2, D1);
  dallassensors.begin();                            //Activates the DS18b20 sensors on the one wire
  numDevices = dallassensors.getDeviceCount();      //Stores the DS18BB20 addresses on the ONEWIRE
  //client.setCallback(callback);                     //Tells the pubsubclient which function to use in case of a callback
  if (!bme.begin(0x76) && !bme.begin(0x77))
  { //This changes the I2C address for the BME280 sensor to the correct one. The Adafruit library expects it to be 0x77 while it is 0x76 for AZ-Delivery articles. Each sensor has to be checked.
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    //while (1)
    //delay(10);
  }
 
}
void loop()
{ //This function will continously be executed; everything which needs to be done recurringly is set here.
  client.loop();
  unsigned long now = millis();
  if (now - lastLoop1 > 2000) {
    lastLoop1 = now;
    connect_wifi_1();
    connect_MQTT_1();
    delay(50);
    measure_temp();
    delay(50);
    measure_humidity();
    delay(50);
    measure_pressure();
    delay(50);
    read_dallas();
    delay(100); //Short delay to finish up all calculations before going to DeepSleep
    client.disconnect(); // disconnect from the MQTT broker
    delay(100); //Short delay to finish up all calculations before going to DeepSleep
    WiFi.disconnect(); // Disconnects the wifi safely
    }
  if(now - lastLoop2 > 2000){
     lastLoop2 = now;
    connect_wifi_2();
    connect_MQTT_2();
    delay(50);
    measure_temp();
    delay(50);
    measure_humidity();
    delay(50);
    measure_pressure();
    delay(50);
    read_dallas();
    delay(100); //Short delay to finish up all calculations before going to DeepSleep
    client.disconnect(); // disconnect from the MQTT broker
    delay(100); //Short delay to finish up all calculations before going to DeepSleep
    WiFi.disconnect(); // Disconnects the wifi safely
  }
}