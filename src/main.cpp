/**
 This code was written/hacked together by Swen Schreiter and is the basis for the multifunctional environmental sensing device called S.E.E.D (Small Electronic Environmental Device).
Project details can be found on GitHub (https://github.com/m31s4d/BEL-ESP-Controller) or at the project blog (TBD). All functionality is released for non-commercial use in the research environment.

 **/

// Include the libraries we need
#include <Arduino.h>
#include <Wire.h> //Adds library for the I2C bus on D1 (SCL) and D2(SCD) (both can be changed on the ESP8266 to the deisred GPIO pins)
//#include <SPI.h>
#include <OneWire.h>           //Adds library needed to initialize and use the 1-Wire protocoll for DS18B20 sensors
#include <DallasTemperature.h> //Adds the Dallas Temp library to use DS18B20 with the Wemos
#include <Adafruit_BME280.h>   //Adds library for the BME280 (Bosch) environmental sensor
#include <Adafruit_Sensor.h>   //Adds library for all Adafruit sensors to make them usable
#include "ESP8266WiFi.h"       // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h"      // Allows us to connect to, and publish to the MQTT broker

#define pH_address 99            //default I2C ID number for EZO pH Circuit.
char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte serial_event = 0;           //a flag to signal when data has been received from the pc/mac/other.
byte response_code = 0;          //used to hold the I2C response code.
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
byte in_char = 0;                //used as a 1 byte buffer to store inbound bytes from the pH Circuit.
byte i = 0;                      //counter used for ph_data array.
int time_1 = 815;                //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float atlas_scientific_ph;       //float var used to hold the float value of the pH.

//Initialization of the OneWire Bus und the temp sensor
const int oneWireBus = D3;                 // GPIO where the DS18B20 is connected to D3
int numDevices;                            // Number of temperature devices found (will be used to get and publish all readings)
OneWire oneWire(oneWireBus);               // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature dallassensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress tempDeviceAddress;           // We'll use this variable to store a found device address for the DS18B20

//Initialization of all environmental variables as global to share them between functions
float bme280_temp = 0; //Sets the variable temp to the temp measure of the BME280
//float bme280_temp2 = 0;     //Sets the variable temp to the temp measure of the BME280
float bme280_humidity = 0; //Sets variable bme_humidity to humidity measure of BME280
//float bme280_humidity2 = 0; //Sets variable bme_humidity to humidity measure of BME280
float bme280_pressure = 0; //Sets variable bme_pressure to pressure measire of BME280
//float bme280_pressure2 = 0; //Sets variable bme_pressure to pressure measire of BME280
float bme280_altitude = 0; //Sets the altitutde variable bme_altitutde to zero
//float bme280_altitude2 = 0; //Sets the altitutde variable bme_altitutde to zero

//Initialization of all environmental variables as global to share them between functions
String bme280_temp_string;     //Sets the variable temp to the temp measure of the BME280
String bme280_humidity_string; //Sets variable bme_humidity to humidity measure of BME280
String bme280_pressure_string; //Sets variable bme_pressure to pressure measire of BME280
String bme280_altitude_string; //Sets the altitutde variable bme_altitutde to zero
//Adafruit_BMP280 bmp; // use I2C interface
Adafruit_BME280 bme; // Create BME280 instance for the first sensor
//Adafruit_BME280 bme2;                                      // Create BME280 Instance for the second sensor
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();  //Gets temperature value from the sensor
Adafruit_Sensor *bme_pressure = bme.getPressureSensor(); //Gets pressure value from sensor
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor(); //Gets humidity value from sensor/ initializes it
//Adafruit_Sensor *bme_temp2 = bme2.getTemperatureSensor();  //Gets temperature value from the sensor 2
//Adafruit_Sensor *bme_pressure2 = bme2.getPressureSensor(); //Gets pressure value from sensor 2
//Adafruit_Sensor *bme_humidity2 = bme2.getHumiditySensor(); //Gets humidity value from sensor2 / initializes it

// MQTT 1 & 2
// These lines initialize the variables for PubSub to connect to the MQTT Broker 1 of the Aero-Table
const char *mqtt_server_1 = "192.168.0.30";    //"192.168.0.111";               //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
const char *mqtt_server_2 = "192.168.178.30";                                     //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
const char *temp_bme280_topic_1 = "aeroponic/growtent1/temperatur/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
//const char *temp_bme280_topic_2 = "aeroponic/growtent1/temperatur/bme280/sensor2";   //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_1 = "aeroponic/growtent1/humidity/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
//const char *humidity_bme280_topic_2 = "aeroponic/growtent1/humidity/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_1 = "aeroponic/growtent1/pressure/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
//const char *pressure_bme280_topic_2 = "aeroponic/growtent1/pressure/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_ds18b20_topic_1 = "aeroponic/growtent1/temperature/d18b20/sensor1";   //Adds MQTT topic for the dallas sensor 1 in the root zone
const char *temp_ds18b20_topic_2 = "aeroponic/growtent1/temperature/d18b20/sensor2";   //Adds MQTT topic for the dallas senssor 2
const char *temp_ds18b20_topic_3 = "aeroponic/growtent1/temperature/d18b20/sensor3";   //Adds MQTT topic for the dallas senssor 3 in the plant zone to measure air temp
const char *pH_ezo_topic_1 = "aeroponic/growtent1/ph/ezo_circuit/sensor1";            //Adds MQTT topic for the AtlasScientific pH probe
const char *pH_command_topic = "aeroponic/growtent1/pH/AtlasScientific/command";  //Adds MQTT topic to subscribe to command code for the EZO pH circuit. With this we will be able remotely calibrate and get readings from the microcontroller
const char *mqtt_connection_topic = "aeroponic/growtent1/connection/table1";          //Adds MQTT topic to check whether the microcontroller is connected to the broker and check the timings
//const char* mqtt_username = "cdavid"; //if MQTT server needs credentials they need to be added in the next two lines
//const char* mqtt_password = "cdavid";

const char *clientID = "ESPTest"; // The client id identifies the ESP8266 device. In this experiment we will use ESP-Table_X-Y (X = Table No, Y = Microcontroller No) Think of it a bit like a hostname (Or just a name, like Greg).
unsigned long noLoop = 0;               //Needed for the deepsleep loop function
unsigned long lastLoop1 = 0;            //Needed for the millis() loop function
unsigned long lastLoop2 = 0;            //Needed for the millis() loop function
unsigned long pHLoop = 0;               //Needed for the millis() of the pH Function to check if 5 minutes are

WiFiClient wifiClient;                                // Initialise the WiFi and MQTT Client objects
PubSubClient client(mqtt_server_1, 1883, wifiClient); // 1883 is the listener port for the Broker //PubSubClient client(espClient);

void connect_wifi(const char *var_ssid, const char *var_wifi_password)
{
  // WiFi connection settings
  //const char *ssid = "Hood Lan";                      //This line needs to be populated by the SSID of the wifi network we want to connect to
  //const char *wifi_password = "Ja17081994Yp08091992"; //This line is populated by the password of the wifi network

  //Defines the wifi connection settings of the second broker
  //const char *ssid = "WLAN-173564";
  //const char *wifi_password = "40568512259452661617";
  // WiFi connection settings
  //const char *ssid = "FRITZ!Box Fon WLAN 7390";                      //This line needs to be populated by the SSID of the wifi network we want to connect to
  //const char *wifi_password = "3830429555162473"; //This line is populated by the password of the wifi network
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(var_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(var_ssid, var_wifi_password);

  while (WiFi.status() != WL_CONNECTED)
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
void connect_MQTT(const char* var_mqtt_client, int port_num)
{ //Defines a function "connect_MQTT" which includes all necessary steps to connect the ESP with the server
  // Create a random client ID
  //String clientId = "SEED-";
  //clientId += String(random(0xffff), HEX);
  client.setServer(var_mqtt_client, port_num); //Important to set the MQTT Server in each connection call, otherwise a connection will not be successful
  PubSubClient client(var_mqtt_client, port_num, wifiClient); // 1883 is the listener port for the Broker
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
void send_data_MQTT(String value, const char *topic)
{
  //strcpy(mqtt_topic, topic.c_str()); // copying the contents of the string to char array
  if (WiFi.status() == WL_CONNECTED)
  {
    //client.connect(clientID);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    if (client.publish(topic, String(value).c_str())) // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    {                        
      Serial.print(value); //To allow debugging a serial output is written if the measurment was published succesfully.
      Serial.println(" sent!");
    }
    // Again, client.publish will return a boolean value depending on whether it succeded or not.
    // If the message failed to send, we will try again, as the connection may have broken.
    else
    {
      Serial.print(value);
      Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(topic, String(value).c_str());
    }
  }
}
/*void measure_soil(){
  //# the approximate moisture levels for the sensor reading
//# 0 to 300 dry soil
//# 300 to 700 humid soil
//# 700 to 950 in water
//#define sensorPin A0
  double sensorValue = analogRead(sensorPin); // read the analog in value:
  sensorValue = map(sensorValue, 1024, 0, 0, 100);
  Serial.print("Moisture : ");
  Serial.println(sensorValue); //Prints out the value of the soil sensor to check if it is wired correctly
  Serial.println("%");
}*/
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
  bme280_temp_string = String((float)bme280_temp); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String humi=String((float)bme280_humidity); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String press=String((float)bme280_pressure); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //String alti=String((float)bme280_altitude); //Important to change the value of the variable to a string
}
void measure_humidity()
{
  sensors_event_t /*temp_event, pressure_event,*/ humidity_event; //Initialization of the sensor events to use them later
  bme_humidity->getEvent(&humidity_event);
  bme280_humidity = bme.readHumidity();         //Sets variable bme_humidity to humidity measure of BME280
  bme280_humidity_string = String((float)bme280_humidity); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //MQTT can only transmit strings
  //send_data_MQTT(humi, humidity_bme280_topic_1);
}
void measure_pressure()
{
  sensors_event_t /*temp_event,*/ pressure_event; /*, humidity_event*/ //Initialization of the sensor events to use them later
  //bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme280_pressure = bme.readPressure(); //Sets variable bme_pressure to pressure measire of BME280
  //bme280_pressure = pressure_event.pressure;     //Sets variable bme_pressure to pressure measire of BME280
  bme280_pressure_string= String((float)bme280_pressure); //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  //MQTT can only transmit strings
  //send_data_MQTT(press, pressure_bme280_topic_1);
}
/*void measure_altitude()
{
#define SEALEVELPRESSURE_HPA (1013.25)                        //Defines the pressure at sea level to calculate the approximate alltitude via the current pressure level
  sensors_event_t temp_event, pressure_event, humidity_event; //Initialization of the sensor events to use them later
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  bme280_temp = temp_event.temperature;               //Sets the variable temp to the temp measure of the BME280
  bme280_humidity = humidity_event.relative_humidity; //Sets variable bme_humidity to humidity measure of BME280
  bme280_pressure = pressure_event.pressure;          //Sets variable bme_pressure to pressure measire of BME280
  float bme280_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  String alti = String((float)bme280_altitude); //Important to change the value of the variable to a string

  send_data_MQTT(alti, altitude_topic);
}*/
/*void measure_pH(String cmd_code)
{
  int code_length = cmd_code.length();    //Gets the length of the string to be used in the for loop later
  strcpy(computerdata, cmd_code.c_str()); // copying the contents of the string to char array
                                          //if a command was sent to the EZO device.
  for (i = 0; i <= code_length; i++)
  {
    computerdata[i] = tolower(computerdata[i]); //"Sleep" ≠ "sleep" //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
  }
  i = 0; //reset i, we will need it later
  if (computerdata[0] == 'c' || computerdata[0] == 'r')
    time_1 = 815; //if a command has been sent to calibrate or take a reading we wait 815ms so that the circuit has time to take the reading.
  else
    time_1 = 250; //if any other command has been sent we wait only 250ms.

  Wire.beginTransmission(pH_address); //call the circuit by its ID number.
  Wire.write(computerdata);           //transmit the command that was sent through the serial port.
  Wire.endTransmission();             //end the I2C data transmission.

  if (strcmp(computerdata, "sleep") != 0)
  { //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
    //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the circuit.
    delay(time_1); //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(pH_address, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
    response_code = Wire.read();         //the first byte is the response code, we read this separately.

    switch (response_code)
    {                            //switch case based on what the response code is.
    case 1:                      //decimal 1.
      Serial.println("Success"); //means the command was successful.
      break;                     //exits the switch case.

    case 2:                     //decimal 2.
      Serial.println("Failed"); //means the command has failed.
      break;                    //exits the switch case.

    case 254:                    //decimal 254.
      Serial.println("Pending"); //means the command has not yet been finished calculating.
      break;                     //exits the switch case.

    case 255:                    //decimal 255.
      Serial.println("No Data"); //means there is no further data to send.
      break;                     //exits the switch case.
    }

    while (Wire.available())
    {                        //are there bytes to receive.
      in_char = Wire.read(); //receive a byte.
      ph_data[i] = in_char;  //load this byte into our array.
      i += 1;                //incur the counter for the array element.
      if (in_char == 0)
      {        //if we see that we have been sent a null command.
        i = 0; //reset the counter i to 0.
        break; //exit the while loop.
      }
    }

    Serial.println(ph_data); //print the data.
  }
  serial_event = false; //reset the serial event flag.
  //Uncomment this section if you want to take the pH value and convert it into floating point number.
  atlas_scientific_ph = atof(ph_data);                           //Converts the array to a float value which we will send via MQTT
  String pH_string = String((float)atlas_scientific_ph);         //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
  if (client.publish(pH_ezo_topic_1, String(pH_string).c_str())) // && client.publish(pressure_bme280_topic_2, String(press2).c_str()) //MQTT can only transmit strings
  {                                                              // PUBLISH to the MQTT Broker (topic was defined at the beginning)
    Serial.print(pH_string + " sent!");                          //To allow debugging a serial output is written if the measurment was published succesfully.
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else
  {
    Serial.println(pH_string);
    Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(pH_ezo_topic_1, String(pH_string).c_str());
  }
}*/
//void ph_serialevent()
//{                                                                       //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.
//received_from_computer = Serial.readBytesUntil(13, computerdata, 20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
//computerdata[received_from_computer] = 0;                             //stop the buffer from transmitting leftovers or garbage.
//serial_event = true;                                                  //set the serial event flag.
//}
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
      if (i == 0)
      {
        //client.publish(temp_ds18b20_topic_1, String(temp_dallas).c_str()); // PUBLISH to the MQTT Broker (topic was defined at the beginning)
        //client.publish(temp_ds18b20_topic_2, String(temp_dallas).c_str());
        send_data_MQTT(temp_dallas, temp_ds18b20_topic_1);
        Serial.println(temp_dallas + " of sensor" + i +"sent!"); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      if (i == 1)
      {
        //client.publish(temp_ds18b20_topic_1, String(temp_dallas).c_str()); // PUBLISH to the MQTT Broker (topic was defined at the beginning)
        //client.publish(temp_ds18b20_topic_2, String(temp_dallas).c_str());
        send_data_MQTT(temp_dallas, temp_ds18b20_topic_2);
        Serial.println(temp_dallas + " of sensor" + i +"sent!"); //To allow debugging a serial output is written if the measurment was published succesfully.
      }
     if (i == 2)
      {
        //client.publish(temp_ds18b20_topic_1, String(temp_dallas).c_str()); // PUBLISH to the MQTT Broker (topic was defined at the beginning)
        //client.publish(temp_ds18b20_topic_2, String(temp_dallas).c_str());
        send_data_MQTT(temp_dallas, temp_ds18b20_topic_3);
        Serial.println(temp_dallas + " of sensor" + i +"sent!"); //To allow debugging a serial output is written if the measurment was published succesfully.
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
  Serial.begin(9600);                          // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();                                // On esp8266 you can select SCL and SDA pins using Wire.begin(D2, D1);
  dallassensors.begin();                       //Activates the DS18b20 sensors on the one wire
  numDevices = dallassensors.getDeviceCount(); //Stores the DS18BB20 addresses on the ONEWIRE
  //client.setCallback(callback);                     //Tells the pubsubclient which function to use in case of a callback
  if (!bme.begin(0x76))
  { //This changes the I2C address for the BME280 sensor to the correct one. The Adafruit library expects it to be 0x77 while it is 0x76 for AZ-Delivery articles. Each sensor has to be checked.
    Serial.println(F("Could not find first BME280 sensor, check wiring!"));
    //while (1)
    //delay(10);
  }
  //if (!bme2.begin(0x77))
  //{ //This changes the I2C address for the BME280 sensor to the correct one. The Adafruit library expects it to be 0x77 while it is 0x76 for AZ-Delivery articles. Each sensor has to be checked.
  // Serial.println(F("Could not find second BME280 sensor, check wiring!"));
  //while (1)
  //delay(10);
  // }
}
void loop()
{ //This function will continously be executed; everything which needs to be done recurringly is set here.
  if (WiFi.status() != WL_CONNECTED)
  {
    connect_wifi("FRITZ!Box Fon WLAN 7390", "3830429555162473");
    delay(500);
    connect_MQTT(mqtt_server_2, 1883);
  }
  unsigned long now = millis();
  if ((now - lastLoop1) > 5000)
  {
    lastLoop1 = now;
    measure_temp();
    measure_humidity();
    measure_pressure();
    read_dallas();
  
    //Serial.print("Disconnecting from MQTT Broker");
    //client.disconnect(); // disconnect from the MQTT broker
    //delay(1000);          //Short delay to finish up all calculations before going to DeepSleep
    //Serial.print("Disconnecting from WiFi");
    //WiFi.disconnect(); // Disconnects the wifi safely
    if(client.connected()){
      send_data_MQTT(bme280_pressure_string, pressure_bme280_topic_1);
      send_data_MQTT(bme280_temp_string, temp_bme280_topic_1);
      send_data_MQTT(bme280_humidity_string, humidity_bme280_topic_1);
    }else{
      Serial.print("Connection to MQTT broker lost/broken! Retrying connection");
      //connect_MQTT(mqtt_server_2, 1883);
    }
  }
  /*if (now - lastLoop2 > 15000)
  {
    lastLoop2 = now;
    //Serial.print("Skipping Connecton 2 for now");
    //connect_wifi("FRITZ!Box Fon WLAN 7390", "3830429555162473");
    delay(1000);
    //connect_MQTT(mqtt_server_2, 1883);
    delay(50);
    measure_temp();
    delay(50);
    measure_humidity();
    delay(50);
    measure_pressure();
    delay(50);
    read_dallas();
    delay(100);          //Short delay to finish up all calculations before going to DeepSleep
    client.disconnect(); // disconnect from the MQTT broker
    delay(1000);          //Short delay to finish up all calculations before going to DeepSleep
    //WiFi.disconnect();   // Disconnects the wifi safely
  }*/

  /*if (now - pHLoop > 300000)
  {
    pHLoop = now;
    //connect_wifi_1();
    //connect_MQTT_1();
    delay(50);
    measure_pH("reading");
    delay(50);
    measure_pH("sleep");
    Serial.print("Disconnecting from MQTT Broker");
    client.disconnect(); // disconnect from the MQTT broker
    delay(100);          //Short delay to finish up all calculations before going to DeepSleep
    Serial.print("Disconnecting from WiFi");
    WiFi.disconnect(); // Disconnects the wifi safely
  }*/
 // client.loop();
}

/*void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String msg;
  if(topic == "aeroponic/growtent1/pH/AtlasScientific/command"){
   
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    msg = msg + (char)payload[i];
  }
  if (msg == "on")
  {
    Serial.println("I received an ON");
  }
  else if (msg == "off")
  {
    Serial.println("I received an OFF");
  }
  }
}*/