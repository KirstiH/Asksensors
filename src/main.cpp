/*
Kirsti Härö
23.11.2023 

This program measures temperature inside, humidity, luminosity and acceleration.
Humidity and inside temperature are measured with SHTC3 sensor, acceleration with LSM6DS3 sensor, luminosity
with TSL2591. If the acceleration is more than 1.5, the red led goes on and it sends 1 to asksensor.com.
The alarm can be stopped by pressing the button and it will send 0 to asksensors.com and alarm is off.

I have used Arduino Nano 33 IoT for this.

The results are dispayed on Asksensors.com with different graphs. 
Outside temperature is imported from api.openweathermap.org.
*/

#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_SHTC3.h>
#include <Arduino_LSM6DS3.h>


// Functions
void printWifiStatus();
void httpRequest();
void mittaaSHT();
void mittaaTSL();
void haeUlkolampotila();
void kuittaus();
void alert();


int led = 13; // led pin
int Button1 = 2; // interrupt button
byte state_led = LOW; // button state
byte previous_state = LOW; // saving the led state

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

uint16_t ir, full; // for TLS sensor value
sensors_event_t humidity, temp; //for humidity and temperature
int lux; //integer for luminosity
float jsonUlkolampotila; //for temperature

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  
  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  Serial.println(F("25x (Medium)"));
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}



char ssid[] = "xxxxxxx";        // your network SSID (name)
char pass[] = "xxxxxxxxx";    // your network password
char apikey1[] = "0gLkXFNQEIuA5JnFGY2rUGGDmQdVd266"; //apikey of first sensor
char apikey2[] = "5ODbIZqE8I5R6um3uTQCLxucNQQeM1AM"; //apikey of second sensor
int keyIndex = 0;            // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

//Json data server and client
char jsonServer[] = "api.openweathermap.org";
WiFiClient jsonClient;

// Initialize the WiFi client library
WiFiClient client;

// server address:
char server[] = "asksensors.com";
unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 60L * 5000L; // delay 5 min between updates, in milliseconds

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; 
  }

  pinMode(led, OUTPUT); // sets the digital pin 13 as output
  pinMode(Button1, INPUT); // sets the digital pin 12 as input
  digitalWrite(led, state_led);

  attachInterrupt(digitalPinToInterrupt(Button1), kuittaus, FALLING);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  //making sure SHTC3 works
  Serial.println("SHTC3 test");
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");

  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  
  if (tsl.begin(0x29)) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

   String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the status:
  printWifiStatus();
}



void loop() {

  float x, y, z, g; // for acceleration measurements

  // if movement is detected or button is pressed,
  // send information to AskSensors and turn on/off led
  if (state_led != previous_state){
    digitalWrite(led, state_led); // turn led on when acceleration is detected
    previous_state = state_led; // save the new state
    alert();
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    g = sqrt(x*x+y*y+z*z);
    if(g>1.5){
      state_led = HIGH; //turn led on when movement is detected
    }
  }
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  
  //After 1 min, get sensor readings and connect to server:
  if ((millis() - lastConnectionTime) > postingInterval) {

    //after 5 minutes read the data again
    mittaaSHT();
    mittaaTSL(); //
    haeUlkolampotila(); // getting outside temperature
    httpRequest(); //request function for all the other sensors
  }

  
}
 

//	This method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");

    String url = "https://asksensors.com/api.asksensors/write/"; //For sensor
    url += apikey1;
    url += "?module1="; //Temperature: line
    url += temp.temperature;
    url += "&module2="; //Humidity: line
    url += humidity.relative_humidity;
    url += "&module3="; //Temperature: table
    url += temp.temperature;
    url += "&module4="; //Humidity: table
    url += humidity.relative_humidity;

    //second sensor
    String url2 = "https://asksensors.com/api.asksensors/write/"; //For sensor
    url2 += apikey2;
    url2 += "?module1="; //Luminosity: line
    url2 += lux;
    url2 += "&module2="; //temperature outside: line
    url2 += jsonUlkolampotila;
    
    // send the HTTP GET request:
    Serial.print("********** requesting URL: ");
      Serial.println(url);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + server + "\r\n");
    client.println();
    
    // send the HTTP GET request for second sensor:
    Serial.print("********** requesting URL: ");
      Serial.println(url2);
    client.print(String("GET ") + url2 + " HTTP/1.1\r\n" +
               "Host: " + server + "\r\n");
    client.println();
  
    
    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// measuring temperature and humidity
void mittaaSHT(){
  shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
}

// measuring luminosity
void mittaaTSL(){
  uint32_t lum = tsl.getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;
  lux = (int)tsl.calculateLux(full, ir); //change to int
}

void haeUlkolampotila() {

  Serial.println("\nStarting connection to JSONserver...");
  // if you get a connection, report back via serial:
  if (jsonClient.connect(jsonServer, 80)) {
    Serial.println("connected to server");
    
    // Send HTTP request:
    jsonClient.println(F("GET /data/2.5/weather?q=Kokkola&appid=3ad47f513f5e580203d5790e72d26e16&units=metric HTTP/1.0")); 
    jsonClient.println(F("Host: openweathermap.org"));
    jsonClient.println(F("Connection: close"));
    if (jsonClient.println() == 0) {
      Serial.println(F("Failed to send request"));
      jsonClient.stop();
      return;
    }
    
    // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!jsonClient.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      jsonClient.stop();
      return;
    }

    // Allocate the JSON document
    // Use https://arduinojson.org/v6/assistant to compute the capacity.
    const size_t capacity = 1024;  //laskettu assistantilla
  
    DynamicJsonDocument doc(capacity);

    // Parse JSON object
    DeserializationError error = deserializeJson(doc, jsonClient);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      jsonClient.stop();
      return;
    }

    // Extract value of outside temperature
    jsonUlkolampotila = doc["main"]["temp"];


    // Disconnect
    jsonClient.stop();
  }
}

// button interrupt state switch
void kuittaus(){
  state_led = LOW;
}

// for detecting movement
void alert(){
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");
  }

  //second sensor
  String url3 = "https://asksensors.com/api.asksensors/write/"; //For sensor
  url3 += apikey2;
  url3 += "?module3="; //movement: binary
  url3 += state_led;
  url3 += "&module4="; //movement: table
  url3 += state_led;

 // send the HTTP GET request for second sensor:
  Serial.print("********** requesting URL: ");
  Serial.println(url3);
  client.print(String("GET ") + url3 + " HTTP/1.1\r\n" +
               "Host: " + server + "\r\n");
  client.println();

}
