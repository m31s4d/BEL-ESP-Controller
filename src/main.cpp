/**
 This code was written/hacked together by Swen Schreiter and is the basis for the multifunctional environmental sensing device called S.E.E.D (Small Electronic Environmental Device).
Project details can be found on GitHub (https://github.com/m31s4d/BEL-ESP-Controller) or at the project blog (TBD). All functionality is released for non-commercial use in a research environment.
 **/
#define HWTYPE 1    // HWTYPE stores which sensors are attached to it: 0=BME280, DS18B20, I2C Multiplexer, 1= pH & EC
#define TENTNO "B2" //Number of research tent either A1/A2/B1/B2/C1/C2

// Include the libraries we need
#include "Arduino.h"
#include "Wire.h"         //Adds library for the I2C bus on D1 (SCL) and D2(SCD) (both can be changed on the ESP8266 to the deisred GPIO pins)
#include "ESP8266WiFi.h"  // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h" // Allows us to connect to, and publish to the MQTT broker
#include <TaskScheduler.h> //Adds multitasking by task scheduling

String nameBuffer = "BEL-Ponic-" + String(ESP.getChipId(), HEX); //String(random(0xffff), HEX); // Create a random client ID to identify the controller in the network
const char *clientID = nameBuffer.c_str();                       //Copies the string in the char array used by the mqtt library

WiFiClient wifiClient;                                   // Initialise the WiFi and MQTT Client objects
PubSubClient client("192.168.178.29", 1883, wifiClient); // 1883 is the listener port for the Broker //PubSubClient client(espClient);
Scheduler tskscheduler;                                  //Initializes a TaskScheduler used to

//Function stubs so TaskScheduler doesnt complain
void startSensors();
Task taskStartSensors(TASK_SECOND, TASK_ONCE, &startSensors);

// MQTT 1 & 2
// These lines initialize the variables for PubSub to connect to the MQTT Broker 1 of the Aero-Table
const char *mqtt_server = "192.168.178.29";                                                       //"192.168.0.111";               //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
String mqtt_connection_topic = "aeroponic/" + String(TENTNO) + "/connection/" + String(clientID); //Adds MQTT topic to check whether the microcontroller is connected to the broker and check the timings

#if HWTYPE == 0                                                                                   //
//#include <SPI.h>
#include "OneWire.h"           //Adds library needed to initialize and use the 1-Wire protocoll for DS18B20 sensors
#include "DallasTemperature.h" //Adds the Dallas Temp library to use DS18B20 with the Wemos
#include "Adafruit_BME280.h"   //Adds library for the BME280 (Bosch) environmental sensor
#include "Adafruit_Sensor.h"   //Adds library for all Adafruit sensors to make them usable
#define TCAADDR 0x70

//Functions stubs so VSCode doesn't complain
void read_bme280();
void read_ds18b20();
void read_soilmoisture();

Task taskBME280(TASK_SECOND * 5, TASK_FOREVER, &read_bme280);
Task taskDS18B20(TASK_SECOND * 5, TASK_FOREVER, &read_ds18b20);
Task taskSoilMoisture(TASK_MINUTE * 10, TASK_FOREVER, &read_soilmoisture);

Adafruit_BME280 bme; // Create BME280 instance for the first sensor

//Initialization of the OneWire Bus und the temp sensor
const int oneWireBus = D3;                 // GPIO where the DS18B20 is connected to D3
int numDevices;                            // Number of temperature devices found (will be used to get and publish all readings)
OneWire oneWire(oneWireBus);               // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature dallassensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress tempDeviceAddress;           // We'll use this variable to store a found device address for the DS18B20

//Initialization of all environmental variables as global to share them between functions
float bme280_temp, bme280_humidity, bme280_pressure, bme280_altitude; //Sets the altitutde variable bme_altitutde to zero
float dallas_temp_0, dallas_temp_1, dallas_temp_2;                    //Sets variable for the first DS18B20 found on the bus

//Initialization of all environmental variables as global to share them between functions
String dallas_temp_0_string, dallas_temp_1_string, String dallas_temp_2_string; //Variable needed for MQTT transmission of DS18B20 measurements

#define sensorPin A0 //Sets analoge pin as sensor input for soilsensor
#define soilPin D5   //Defines D5 as output pin connected to VCC on moisture sensore. Reduces
int soil_moisture;
unsigned long soilLoop = 0; //Needed for the millis() loop function

String temp_bme280_topic_1 = "aeroponic/" + String(TENTNO) + "/temperature/bme280/sensor1";       //Adds MQTT topic for the sensor readings of the aero-grow-tables
String humidity_bme280_topic_1 = "aeroponic/" + String(TENTNO) + "/humidity/bme280/sensor1";      //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_1 = "aeroponic/" + String(TENTNO) + "/pressure/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_bme280_topic_2 = "aeroponic/" + String(TENTNO) + "temperature/bme280/sensor2";   //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_2 = "aeroponic/" + String(TENTNO) + "/humidity/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_2 = "aeroponic/" + String(TENTNO) + "/pressure/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_ds18b20_topic_1 = "aeroponic/" + String(TENTNO) + "/temperature/d18b20/sensor1"; //Adds MQTT topic for the dallas sensor 1 in the root zone
const char *temp_ds18b20_topic_2 = "aeroponic/" + String(TENTNO) + "/temperature/d18b20/sensor2"; //Adds MQTT topic for the dallas senssor 2
const char *temp_ds18b20_topic_3 = "aeroponic/" + String(TENTNO) + "/temperature/d18b20/sensor3"; //Adds MQTT topic for the dallas senssor 3 in the plant zone to measure air temp
String soil_moisture_topic = "aeroponic/" + String(TENTNO) + "/soil_moisture/sensor1";            //Adds MQTT topic to check whether the microcontroller is connected to the broker and check the timings
#else

#include <Ezo_i2c.h>      //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include "Ezo_i2c_util.h" //brings in common print statements
#include <U8g2lib.h>      //Needed for OLED Screen

Ezo_board PH = Ezo_board(99, "PH");  //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC"); //create an EC circuit object who's address is 100 and name is "EC"

U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(/* clock=A5*/ D1, /* data=A4*/ D2); //Defines the type of oled display used later on. In this case 128x32

float ph_float; //float var used to hold float of pH value
float ec_float; //float var used to hold the float value of the specific gravity.

//Function stubs for TaskScheduler so VSCode doesn't complain
void read_EC();
void parse_EC();
void read_PH();
void parse_PH();
void print_oled();

//Initialize task to read/parse EC & pH and print to OLED screen
Task taskReadEC(TASK_SECOND * 31, TASK_FOREVER, &read_EC);
Task taskParseEC(TASK_SECOND * 36, TASK_FOREVER, &parse_EC);
Task taskReadPH(TASK_MINUTE, TASK_FOREVER, &read_PH);
Task taskParsePH(TASK_SECOND * 66, TASK_FOREVER, &parse_PH);
Task taskPrintOLED(TASK_SECOND *5, TASK_FOREVER, &print_oled);

//MQTT: Include the following topics to send data value correctly for pH and EC
String pH_ezo_topic_1 = "aeroponic/" + String(TENTNO) + "/ph/sensor1";   //Adds MQTT topic for the AtlasScientific pH probe
String pH_command_topic = "aeroponic/" + String(TENTNO) + "/ph/command"; //Adds MQTT topic to subscribe to command code for the EZO pH circuit. With this we will be able remotely calibrate and get readings from the microcontroller
String ec_ezo_topic_1 = "aeroponic/" + String(TENTNO) + "/ec/sensor1";   //Adds MQTT topic for the AtlasScientific pH probe
String ec_command_topic = "aeroponic/" + String(TENTNO) + "/ec/command"; //Adds MQTT topic to subscribe to command code for the EZO pH circuit. With this we will be able remotely calibrate and get readings from the microcontroller
#endif
void connect_wifi(const char *var_ssid, const char *var_wifi_password)
{
  //Defines the wifi connection settings of the second broker
  //const char *ssid = "WLAN-173564";
  //const char *wifi_password = "40568512259452661617";
  // WiFi connection settings
  //const char *ssid = "FRITZ!Box Fon WLAN 7390";                      //This line needs to be populated by the SSID of the wifi network we want to connect to
  //const char *wifi_password = "3830429555162473"; //This line is populated by the password of the wifi network
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(var_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(var_ssid, var_wifi_password);

  while (WiFi.status() != WL_CONNECTED) //As long as WiFi is not connected wait 500 millisecs
  {
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
void connect_MQTT(const char *var_mqtt_client, int port_num)
{ //Defines a function "connect_MQTT" which includes all necessary steps to connect the ESP with the server
  //PubSubClient client(var_mqtt_client, port_num, wifiClient); // 1883 is the listener port for the Broker
  client.setServer(var_mqtt_client, port_num); //Important to set the MQTT Server in each connection call, otherwise a connection will not be successful
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID))
  {
    Serial.println("Connected to MQTT Broker!");
    //MQTT Topics to subscribe to for functioning. Structure as follows: project/location/type/number
    //Following are test topics
    //delay(100);
    //client.publish(mqtt_connection_topic_1, "on");
    //client.subscribe("aeroponic/B1/EC/command");
  }
  else
  {
    Serial.println("Connection to MQTT Broker failed...");
  }
}
void send_data_MQTT(String value, String topic)
{
  //strcpy(mqtt_topic, topic.c_str()); // copying the contents of the string to char array
  if (WiFi.status() == WL_CONNECTED)
  {
    if (client.publish(topic.c_str(), String(value).c_str())) // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    {
      Serial.print(value); //To allow debugging a serial output is written if the measurment was published succesfully.
      Serial.println("sent to topic:");
      Serial.println(topic);
    }
    // Again, client.publish will return a boolean value depending on whether it succeded or not.
    // If the message failed to send, we will try again, as the connection may have broken.
    else
    {
      Serial.print(value);
      Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(topic.c_str(), String(value).c_str());
    }
  }
}
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  String tmp_topic = topic;
  char msg[length];
  if (tmp_topic == pH_command_topic)
  {
    Serial.print("Message:");
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
    }
    PH.send_cmd(msg);
  }
  if (tmp_topic == ec_command_topic)
  {
    Serial.print("Message:");
    for (int i = 0; i < length; i++)
    {
      //Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
    }
    Serial.print(msg);
  }
}
void startSensors()
{
#if HWTYPE == 0
  dallassensors.begin();                       //Activates the DS18b20 sensors on the one wire
  numDevices = dallassensors.getDeviceCount(); //Stores the DS18BB20 addresses on the ONEWIRE

  if (!bme.begin(0x76))
  { //This changes the I2C address for the BME280 sensor to the correct one. The Adafruit library expects it to be 0x77 while it is 0x76 for AZ-Delivery articles. Each sensor has to be checked.
    Serial.println(F("Could not find first BME280 sensor, check wiring!"));
    //while (1)
    //delay(10);
  }
  Serial.print("HWTYPE is 0, BME280 and DS18B20 sensors need to be started!");
  if (taskStartSensors.isFirstIteration())
  {
    tskscheduler.addTask(taskBME280);
    taskBME280.enable();
    tskscheduler.addTask(taskDS18B20);
    taskDS18B20.enable();
  }
#else
  Serial.print("HWTYPE is 1, pH & EC sensors need to be started!");
  if (taskStartSensors.isFirstIteration())
  {
    tskscheduler.addTask(taskReadEC); //Adds and enables the task to read EC values from probe
    taskReadEC.enable();
    tskscheduler.addTask(taskReadPH); //Adds and enables the task to read pH values from probe
    taskReadPH.enable();
    tskscheduler.addTask(taskPrintOLED); //Adds and enables the task to print values to OLED
    taskPrintOLED.enable();
  }
#endif
}
#if HWTYPE == 0
void tca_bus_select(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void measure_bme280(int tca_bus)
{
  tca_bus_select(tca_bus);
#define SEALEVELPRESSURE_HPA (1013.25) //Defines the pressure at sea level to calculate the approximate alltitude via the current pressure level
  bme280_temp = bme.readTemperature(); //Sets the variable temp to the temp measure of the BME280
  Serial.print(bme280_temp);
  bme280_humidity = bme.readHumidity(); //Sets variable bme_humidity to humidity measure of BME280
  Serial.print(bme280_humidity);
  bme280_pressure = bme.readPressure(); //Sets variable bme_pressure to pressure measire of BME280
  Serial.print(bme280_pressure);
  //float bme280_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}
void read_bme280()
{
  measure_bme280(0);
  send_data_MQTT(String(bme280_temp), String(temp_bme280_topic_1));
  send_data_MQTT(String(bme280_humidity), String(humidity_bme280_topic_1));
  send_data_MQTT(String(bme280_pressure), String(pressure_bme280_topic_1));
  measure_bme280(7);
  send_data_MQTT(String(bme280_temp), temp_bme280_topic_2);
  send_data_MQTT(String(bme280_humidity), humidity_bme280_topic_2);
  send_data_MQTT(String(bme280_pressure), pressure_bme280_topic_2);
}
void measure_ds18b20()
{
  dallassensors.requestTemperatures(); //Sends a request to the DS18B20 to prepare a temperature reading
  // Loop through each device, print out temperature data
  for (int i = 0; i < numDevices; i++)
  {
    // Search the wire for address
    if (dallassensors.getAddress(tempDeviceAddress, i))
    {
      if (i == 0)
      {
        // Output the device ID
        Serial.print("Temperature for device: ");
        Serial.println(i, DEC);

        // Print the data
        dallas_temp_0 = dallassensors.getTempC(tempDeviceAddress);
        Serial.print("Temp C: ");
        Serial.print(dallas_temp_0);
        Serial.println(String(dallas_temp_0) + " of sensor " + i); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      if (i == 1)
      {
        // Output the device ID
        Serial.print("Temperature for device: ");
        Serial.println(i, DEC);

        // Print the data
        dallas_temp_1 = dallassensors.getTempC(tempDeviceAddress);
        Serial.println(String(dallas_temp_1) + " of sensor" + i + "received!"); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
      if (i == 2)
      {
        // Output the device ID
        Serial.print("Temperature for device: ");
        Serial.println(i, DEC);

        // Print the data
        dallas_temp_2 = dallassensors.getTempC(tempDeviceAddress);
        Serial.println(String(dallas_temp_2) + " of sensor" + i + "received!"); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
    }
    //float temperatureC = dallassensors.getTempCByIndex(0); //Adds the value of the 0 device of temperature to the variable tempc
  }
}
void read_ds18b20()
{
  measure_ds18b20();
  send_data_MQTT(String(dallas_temp_0, temp_ds18b20_topic_1));
  send_data_MQTT(String(dallas_temp_1, temp_ds18b20_topic_2));
  send_data_MQTT(String(dallas_temp_2, temp_ds18b20_topic_3));
}
void measure_soil()
{
  //# the approximate moisture levels for the sensor reading
  //# 0 to 300 dry soil
  //# 300 to 700 humid soil
  //# 700 to 950 in water
  pinMode(soilPin, OUTPUT);
  digitalWrite(soilPin, HIGH);
  double sensorValue = analogRead(sensorPin); // read the analog in value:
  sensorValue = map(sensorValue, 1024, 0, 0, 100);
  Serial.print("Moisture : ");
  Serial.println(sensorValue); //Prints out the value of the soil sensor to check if it is wired correctly
  soil_moisture = sensorValue;
  Serial.println("%");
}
#else
void read_PH()
{
  if (taskReadPH.isFirstIteration()) //During first iteration parsePH task needs to be enabled!
  {                                  //Enables the parsing/sending of pH values
    tskscheduler.addTask(taskParsePH);
    taskParsePH.enable();
  }
  PH.send_read_cmd(); //EZO circuit needs 1s to take a reading, sending a value in a different task than request solves problems.
}
void parse_PH()
{
  if (PH.is_read_poll())
  {
    PH.receive_read_cmd();                     //get the response data and put it into the [Sensor].reading variable if successful
    ph_float = PH.get_last_received_reading(); //Puts the last received ph value in the ph_float variable
    Serial.print("pH: ");                      //Debugging
    Serial.println(ph_float);
    Serial.print("");
    if (ph_float > 0) //Only if we have proper vlaues (over 0) we transmit via mqtt
    {
      send_data_MQTT(String(ph_float), String(pH_ezo_topic_1));
    }
  }
}
void read_EC() //Reading and requesting data from EZO circuits need to be split up. Otherwise board is not ready and reading will be 0.
{ //Values have a variation of +/- 50 µS/cm
  //Ezo circuit requires pull-up resistors on SDA and SCL (not for Wemos D1 mini! Also only stable with 5V!
  if (taskReadEC.isFirstIteration()) // During first iteration the parse EC task needs to be added and enabled!
  {
    tskscheduler.addTask(taskParseEC);
    taskParseEC.enable();
  }
  EC.send_read_cmd(); //Sends the "R" command to EZO circuit
}
void parse_EC()
{
  EC.receive_read_cmd(); //Requests the last reading saved in the EZO circuit
  if (EC.is_read_poll()) //Necessary to see if EZO is ready for reading out the value
  {
    //Serial.print("Error type: ");
    //Serial.println(EC.get_error());
    ec_float = EC.get_last_received_reading();
    Serial.print("EC: "); //Debuuging
    Serial.println(ec_float);
    Serial.print("");
    if (ec_float > 0)
    {
      send_data_MQTT(String(ec_float), String(ec_ezo_topic_1));
    }
  }
}
void print_oled()
{
  u8x8.setFont(u8x8_font_chroma48medium8_r); //Sets the font the oled uses. NECESSARY: Without is null pointer
  u8x8.drawString(0, 1, "EC:");
  u8x8.drawString(3, 1, String(ec_float).c_str());
  u8x8.drawString(0, 2, "pH:");
  u8x8.drawString(3, 2, String(ph_float).c_str());
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
#endif
void setup()
{
  Serial.begin(9600);                        // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();                              // On esp8266 you can select SCL and SDA pins using Wire.begin(D2, D1);
  client.setCallback(mqtt_callback);         //Tells the pubsubclient which function to use in case of a callback
  u8x8.begin();                              //Initializes the u8x8 oled display
  u8x8.setPowerSave(0);                      //Removes the power saving from the oled
  u8x8.setFont(u8x8_font_chroma48medium8_r); //Specifies the used font on the display. Needed otherwise null pointer!

  tskscheduler.addTask(taskStartSensors); //Adds task to scheduler list
  taskStartSensors.enable();              //Enables the start sensors task
}
void loop()
{ //This function will continously be executed; everything which needs to be done recurringly is set here.
  //unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED)
  {
    connect_wifi("FRITZ!Box Fon WLAN 7390", "3830429555162473");
    //connect_wifi("Hood Lan","Ja17081994Yp08091992");
  }
  if (!client.connected())
  {
    connect_MQTT(mqtt_server, 1883);
  }
  tskscheduler.execute();
  client.loop();
}