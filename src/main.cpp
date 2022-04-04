#include <Arduino.h>

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>

#include <Adafruit_BMP280.h>
#include <SPI.h>

#include "RTClib.h"

RTC_DS3231 rtc;


#define SEALEVELPRESSURE_HPA (1013.25)

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);


// Replace with your network credentials
const char* ssid     = "237HomeTyga";
const char* password = "Overdose2021";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

String dateH;

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;



// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 Forced Mode Test."));

   // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);


  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

 /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
    #ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

 if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");

    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){

  DateTime now = rtc.now();
  dateH ="";
   
    dateH += String(now.year(), DEC); 
    dateH += '/';
    dateH += String(now.month(), DEC);
    dateH += '/';
    dateH += String(now.day(), DEC);
    dateH += (" (");
    dateH += String(daysOfTheWeek[now.dayOfTheWeek()]);
    dateH += (") ");
    dateH += String(now.hour(), DEC);
    dateH += (':');
    dateH += String(now.minute(), DEC);
    dateH += (':');
    dateH += String(now.second(), DEC);
    Serial.println(dateH);
    Serial.println();
    

if (bmp.takeForcedMeasurement()) {
    
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.println();
    delay(2000);
  } else {
    Serial.println("Forced measurement failed!");
  }


  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link href=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css\" rel=\"stylesheet\" integrity=\"sha384-1BmE4kWBq78iYhFldvKuhfTAU6auU8tT94WrHftjDbrCEXSU1oBoqyl2QvZ6jIW3\" crossorigin=\"anonymous\">");
            client.println("<script src=\"https://kit.fontawesome.com/7bd22ada10.js\" crossorigin=\"anonymous\"></script>");
            // CSS to style the table 
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial;}");
            client.println(".divbutton { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button1 { background-color: #da2e30; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 { background-color: #33a3d7; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button12 {background-color: #811718;}");
            client.println(".button22 {background-color: #242b6b;}");
            client.println("table { border-collapse: collapse; width:35%; margin-left:auto; margin-right:auto; }");
            client.println("th { padding: 12px; background-color: rgb(22, 146, 146); color: rgb(22, 146, 146);  }");
            client.println("tr { border: 1px solid #ddd; padding: 12px; }");
            client.println("tr:hover { background-color: #bcbcbc; }");
            client.println("td { border: none; padding: 12px; color: rgb(22, 146, 146);}");
            client.println("container{margin: 20px auto; width: 20%; padding: 15px;}");
            
            // Web Page Heading
            client.println("</style></head><body><h1>ESP32 BMP280 SERVER</h1>");
            client.println("<div class=\"container\"><table class=\"table table-borderless\"><thead><tr><th scope=\"col\">Data</th><th scope=\"col\">LastUpdate</th><th scope=\"col\">Value</th></tr></thead>");
            client.println("<tbody><tr><td><i class=\"fa-solid fa-temperature-half fa-3x color2 icone\"></i></td><td>");
            client.println(dateH);
            client.println("</td><td>");
            client.println(bmp.readTemperature());
            client.println("<sup>°C </sup></td></tr></tbody></table></div> ");
            client.println("<div class=\"divbutton\"><p>CHAUFFAGE - State " + output26State + "</p> ");
                   
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button1  button12\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button1 \">ON</button></a></p>");
            } 
            client.println("<p>CLIMATISATION - State " + output27State + "</p>");       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button2 button22\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button2 \">ON</button></a></p>");
            }
            client.println("</div></body></html>");

            
           
            client.println();
          
            break;
          } else { 
            currentLine = "";
          }
        } else if (c != '\r') {  
          currentLine += c;      
        }
      }
    
     }
  } 
}
    