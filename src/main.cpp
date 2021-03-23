/**
 This code was written/hacked together by Swen Schreiter and is the basis for the multifunctional environmental sensing device called S.E.E.D (Small Electronic Environmental Device).
Project details can be found on GitHub (https://github.com/m31s4d/BEL-ESP-Controller) or at the project blog (TBD). All functionality is released for non-commercial use in a research environment.

 **/

// Include the libraries we need
#include "Arduino.h"
#include "Wire.h" //Adds library for the I2C bus on D1 (SCL) and D2(SCD) (both can be changed on the ESP8266 to the deisred GPIO pins)
//#include <SPI.h>
#include "OneWire.h"           //Adds library needed to initialize and use the 1-Wire protocoll for DS18B20 sensors
#include "DallasTemperature.h" //Adds the Dallas Temp library to use DS18B20 with the Wemos
#include "Adafruit_BME280.h"   //Adds library for the BME280 (Bosch) environmental sensor
#include "Adafruit_Sensor.h"   //Adds library for all Adafruit sensors to make them usable
#include "ESP8266WiFi.h"       // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h"      // Allows us to connect to, and publish to the MQTT broker

#define TCAADDR 0x70
#define sensorPin A0
#define soilPin D5 //Defines D5 as output pin connected to VCC on moisture sensore. Reduces

#define pH_address 99               //default I2C ID number for EZO pH Circuit.
char ph_computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte ph_received_from_computer = 0; //we need to know how many characters have been received.
byte ph_serial_event = 0;           //a flag to signal when data has been received from the pc/mac/other.
byte ph_response_code = 0;          //used to hold the I2C response code.
char ph_data[20];                   //we make a 20 byte character array to hold incoming data from the pH circuit.
byte ph_in_char = 0;                //used as a 1 byte buffer to store inbound bytes from the pH Circuit.
byte ph_counter = 0;                //counter used for ph_data array.
int time_ph = 815;                  //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float atlas_scientific_ph;          //float var used to hold the float value of the pH.

#define ec_address 100              //default I2C ID number for EZO EC Circuit.
char ec_computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte ec_received_from_computer = 0; //we need to know how many characters have been received.
byte serial_event = 0;              //a flag to signal when data has been received from the pc/mac/other.
byte ec_response_code = 0;          //used to hold the I2C response code.
char ec_data[32];                   //we make a 32 byte character array to hold incoming data from the EC circuit.
byte ec_in_char = 0;                //used as a 1 byte buffer to store inbound bytes from the EC Circuit.
byte ec_counter = 0;                //counter used for ec_data array.
int time_ec = 570;                  //used to change the delay needed depending on the command sent to the EZO Class EC Circuit.

char *ec;  //char pointer used in string parsing.
char *tds; //char pointer used in string parsing.
char *sal; //char pointer used in string parsing.
char *sg;  //char pointer used in string parsing.

float ec_float;  //float var used to hold the float value of the conductivity.
float tds_float; //float var used to hold the float value of the TDS.
float sal_float; //float var used to hold the float value of the salinity.
float sg_float;  //float var used to hold the float value of the specific gravity.

//Initialization of the OneWire Bus und the temp sensor
const int oneWireBus = D3;                 // GPIO where the DS18B20 is connected to D3
int numDevices;                            // Number of temperature devices found (will be used to get and publish all readings)
OneWire oneWire(oneWireBus);               // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature dallassensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress tempDeviceAddress;           // We'll use this variable to store a found device address for the DS18B20

//Initialization of all environmental variables as global to share them between functions
float bme280_temp;     //Sets the variable temp to the temp measure of the BME280
float bme280_humidity; //Sets variable bme_humidity to humidity measure of BME280
float bme280_pressure; //Sets variable bme_pressure to pressure measire of BME280
float bme280_altitude; //Sets the altitutde variable bme_altitutde to zero
float dallas_temp_0;   //Sets variable for the first DS18B20 found on the bus
float dallas_temp_1;   //Sets variable for the first DS18B20 found on the bus
float dallas_temp_2;   //Sets variable for the first DS18B20 found on the bus
int soil_moisture;
//Initialization of all environmental variables as global to share them between functions
String dallas_temp_0_string;       //Variable needed for MQTT transmission of DS18B20 measurements
String dallas_temp_1_string;       //Variable needed for MQTT transmission of DS18B20 measurements
String dallas_temp_2_string;       //Variable needed for MQTT transmission of DS18B20 measurements
String atlas_scientific_ph_string; //Variable to store pH value for MQTT transmission

Adafruit_BME280 bme; // Create BME280 instance for the first sensor

// MQTT 1 & 2
// These lines initialize the variables for PubSub to connect to the MQTT Broker 1 of the Aero-Table
const char *mqtt_server = "192.168.178.29";                                          //"192.168.0.111";               //Here the IP address of the mqtt server needs to be added. HoodLan = 192.168.2.105
const char *temp_bme280_topic_1 = "aeroponic/growtent2/temperature/bme280/sensor1";  //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_1 = "aeroponic/growtent2/humidity/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_1 = "aeroponic/growtent2/pressure/bme280/sensor1"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_bme280_topic_2 = "aeroponic/growtent2/temperature/bme280/sensor2";  //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *humidity_bme280_topic_2 = "aeroponic/growtent2/humidity/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *pressure_bme280_topic_2 = "aeroponic/growtent2/pressure/bme280/sensor2"; //Adds MQTT topic for the sensor readings of the aero-grow-tables
const char *temp_ds18b20_topic_1 = "aeroponic/growtent2/temperature/d18b20/sensor1"; //Adds MQTT topic for the dallas sensor 1 in the root zone
const char *temp_ds18b20_topic_2 = "aeroponic/growtent2/temperature/d18b20/sensor2"; //Adds MQTT topic for the dallas senssor 2
const char *temp_ds18b20_topic_3 = "aeroponic/growtent2/temperature/d18b20/sensor3"; //Adds MQTT topic for the dallas senssor 3 in the plant zone to measure air temp
const char *pH_ezo_topic_1 = "aeroponic/growtent2/ph/ezo_circuit/sensor1";           //Adds MQTT topic for the AtlasScientific pH probe
const char *pH_command_topic = "aeroponic/growtent2/pH/AtlasScientific/command";     //Adds MQTT topic to subscribe to command code for the EZO pH circuit. With this we will be able remotely calibrate and get readings from the microcontroller
const char *mqtt_connection_topic = "aeroponic/growtent2/connection/sensor1";        //Adds MQTT topic to check whether the microcontroller is connected to the broker and check the timings
const char *soil_moisture_topic = "aeroponic/growtent2/soil_moisture/sensor1";       //Adds MQTT topic to check whether the microcontroller is connected to the broker and check the timings

String nameBuffer = "BEL-Ponic-" + String(ESP.getChipId(), HEX); //String(random(0xffff), HEX); // Create a random client ID
const char *clientID = nameBuffer.c_str();
//const char *clientID = "ESP-Table1_Test_1"; // The client id identifies the ESP8266 device. In this experiment we will use ESP-Table_X-Y (X = Table No, Y = Microcontroller No) Think of it a bit like a hostname (Or just a name, like Greg).
unsigned long mainLoop = 0; //Needed for the millis() loop function
unsigned long soilLoop = 0; //Needed for the millis() loop function
unsigned long pHLoop = 0;   //Needed for the millis() of the pH Function to check if 5 minutes are over

WiFiClient wifiClient;                              // Initialise the WiFi and MQTT Client objects
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker //PubSubClient client(espClient);

void tca_bus_select(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}

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
{                                                             //Defines a function "connect_MQTT" which includes all necessary steps to connect the ESP with the server
  client.setServer(var_mqtt_client, port_num);                //Important to set the MQTT Server in each connection call, otherwise a connection will not be successful
  PubSubClient client(var_mqtt_client, port_num, wifiClient); // 1883 is the listener port for the Broker
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
    if (client.publish(topic, String(value).c_str())) // PUBLISH to the MQTT Broker (topic was defined at the beginning)
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
      client.publish(topic, String(value).c_str());
    }
  }
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
/*void measure_pH(String cmd_code)
{
  int code_length = cmd_code.length();    //Gets the length of the string to be used in the for loop later
  strcpy(ph_computerdata, cmd_code.c_str()); // copying the contents of the string to char array
                                          //if a command was sent to the EZO device.
  for (i = 0; i <= code_length; i++)
  {
    ph_computerdata[i] = tolower(ph_computerdata[i]); //"Sleep" ≠ "sleep" //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
  }
  i = 0; //reset i, we will need it later
  if (ph_computerdata[0] == 'c' || ph_computerdata[0] == 'r')
    time_ph = 815; //if a command has been sent to calibrate or take a reading we wait 815ms so that the circuit has time to take the reading.
  else
    time_ph = 250; //if any other command has been sent we wait only 250ms.

  Wire.beginTransmission(pH_address); //call the circuit by its ID number.
  Wire.write(ph_computerdata);           //transmit the command that was sent through the serial port.
  Wire.endTransmission();             //end the I2C data transmission.

  if (strcmp(ph_computerdata, "sleep") != 0)
  { //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
    //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the circuit.
    delay(time_ph); //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(pH_address, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
    ph_response_code = Wire.read();         //the first byte is the response code, we read this separately.

    switch (ph_response_code)
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
      ph_in_char = Wire.read(); //receive a byte.
      ph_data[i] = ph_in_char;  //load this byte into our array.
      i += 1;                //incur the counter for the array element.
      if (ph_in_char == 0)
      {        //if we see that we have been sent a null command.
        i = 0; //reset the counter i to 0.
        break; //exit the while loop.
      }
    }

    Serial.println(ph_data); //print the data.
  }
  ph_serial_event = false; //reset the serial event flag.
  //Uncomment this section if you want to take the pH value and convert it into floating point number.
  atlas_scientific_ph = atof(ph_data);                           //Converts the array to a float value which we will send via MQTT
  String pH_string = String((float)atlas_scientific_ph);         //Important here is that only the value of the measurement is stored in the string. Mycodo automatically converts string-values to float, therefore only the value is allowed to be stored here.
}*/
/*void measure_EC(String cmd_code)
{
   int ec_code_length = cmd_code.length();    //Gets the length of the string to be used in the for loop later
  strcpy(ec_computerdata, cmd_code.c_str()); // copying the contents of the string to char array
  for (int i = 0; i <= ec_received_from_computer; i++)
  {                                                   //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
    ec_computerdata[i] = tolower(ec_computerdata[i]); //"Sleep" ≠ "sleep"
  }
  int i = 0; //reset i, we will need it later
  if (ec_computerdata[0] == 'c' || ec_computerdata[0] == 'r')
    time_ec = 570; //if a command has been sent to calibrate or take a reading we wait 570ms so that the circuit has time to take the reading.
  else
    time_ec = 250; //if any other command has been sent we wait only 250ms.

  Wire.beginTransmission(ec_address); //call the circuit by its ID number.
  Wire.write(ec_computerdata);        //transmit the command that was sent through the serial port.
  Wire.endTransmission();             //end the I2C data transmission.

  if (strcmp(ec_computerdata, "sleep") != 0)
  {                 //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                    //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the EC circuit.
    delay(time_ec); //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(ec_address, 32, 1); //call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
    ec_response_code = Wire.read();      //the first byte is the response code, we read this separately.
    switch (ec_response_code)
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
    {                           //are there bytes to receive.
      ec_in_char = Wire.read(); //receive a byte.
      ec_data[i] = ec_in_char;  //load this byte into our array.
      i += 1;                   //incur the counter for the array element.
      if (ec_in_char == 0)
      {        //if we see that we have been sent a null command.
        i = 0; //reset the counter i to 0.
        break; //exit the while loop.
      }
    }

    Serial.println(ec_data); //print the data.
    Serial.println();        //this just makes the output easier to read by adding an extra blank line
  }

  if (ec_computerdata[0] == 'r')
    ec_string_pars(ec_data); //uncomment this function if you would like to break up the comma separated string into its individual parts.
}
void ec_string_pars(char *ec_data_pars)
{ //this function will break up the CSV string into its 4 individual parts. EC|TDS|SAL|SG.
  //this is done using the C command “strtok”.

  ec = strtok(ec_data_pars, ","); //let's pars the string at each comma.
  tds = strtok(NULL, ",");        //let's pars the string at each comma.
  sal = strtok(NULL, ",");        //let's pars the string at each comma.
  sg = strtok(NULL, ",");         //let's pars the string at each comma.

  Serial.print("EC:"); //we now print each value we parsed separately.
  Serial.println(ec);  //this is the EC value.

  Serial.print("TDS:"); //we now print each value we parsed separately.
  Serial.println(tds);  //this is the TDS value.

  Serial.print("SAL:"); //we now print each value we parsed separately.
  Serial.println(sal);  //this is the salinity value.

  Serial.print("SG:"); //we now print each value we parsed separately.
  Serial.println(sg);  //this is the specific gravity.
  Serial.println();    //this just makes the output easier to read by adding an extra blank line

  //uncomment this section if you want to take the values and convert them into floating point number.
  
    ec_float=atof(ec);
    tds_float=atof(tds);
    sal_float=atof(sal);
    sg_float=atof(sg);

}*/
//void ph_serialevent()
//{                                                                       //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.
//received_from_computer = Serial.readBytesUntil(13, computerdata, 20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
//computerdata[received_from_computer] = 0;                             //stop the buffer from transmitting leftovers or garbage.
//serial_event = true;                                                  //set the serial event flag.
//}
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
}
void loop()
{ //This function will continously be executed; everything which needs to be done recurringly is set here.
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED)
  {
    connect_wifi("FRITZ!Box Fon WLAN 7390", "3830429555162473");
  }
  if (!client.connected())
  {
    connect_MQTT(mqtt_server, 1883);
  }
  if ((now - mainLoop) > 5000)
  {
    mainLoop = now;
    measure_bme280(0);
    send_data_MQTT(String(bme280_temp), temp_bme280_topic_1);
    send_data_MQTT(String(bme280_humidity), humidity_bme280_topic_1);
    send_data_MQTT(String(bme280_pressure), pressure_bme280_topic_1);
    measure_bme280(7);
    send_data_MQTT(String(bme280_temp), temp_bme280_topic_2);
    send_data_MQTT(String(bme280_humidity), humidity_bme280_topic_2);
    send_data_MQTT(String(bme280_pressure), pressure_bme280_topic_2);
    measure_ds18b20();
    send_data_MQTT(String(dallas_temp_0), temp_ds18b20_topic_1);
    send_data_MQTT(String(dallas_temp_1), temp_ds18b20_topic_2);
    //send_data_MQTT(String(dallas_temp_2), temp_ds18b20_topic_3);
  }
  /*if ((now - soilLoop) > 600000)
  {
    soilLoop = now;
    measure_soil();
//send_data_MQTT(String(soil_moisture), soil_moisture_topic);
  }*/
  /*if (now - pHLoop > 3000000)
  {
    pHLoop = now;
    measure_pH("reading");
    send_data_MQTT(atlas_scientific_ph_string, pH_ezo_topic_1);
    delay(50);
    measure_pH("sleep");
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