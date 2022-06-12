// Q1 fab Quadruped Robot (Designed by Jason Leung)
//
// Q1 fab
// Project website: jasonworkshop.com/q1fab
//
// Jason Workshop
// Website: http://jasonworkshop.com
//
// Firmware version 0.0.3
// Last Update: 4 Jun 2022
//
//
// Q1 fab (Top View)
//
// Servo                           Sensor
//  -----               -----      -----               -----
// |  5  |             |  1  |    |  5  |             |  1  |
// | P23 |             | P18 |    | P34 |             | P35 |
//  ----- -----   ----- -----      ----- -----   ----- -----
//       |  6  | |  2  |                |  6  | |  2  |
//       | P22 | | P17 |                | P25 | | P26 |
//        -----   -----                  -----   -----
//       |  7  | |  3  |                |  7  | |  3  |
//       | P21 | | P16 |                | P27 | | P12 |
//  ----- -----   ----- -----      ----- -----   ----- -----
// |  8  |             |  4  |    |  8  |             |  4  |
// | P19 |             | P04 |    | P32 |             | P33 |
//  -----               -----      -----               -----
//
// ----------------------------------------------------------------------------------------------------
//
// This Firmware licensed under the Attribution-NonCommercial-ShareAlike 4.0 (CC-BY-NC-SA 4.0)
//
// Attribution: You must give appropriate credit, provide a link to the license, and indicate if changes were made.
// You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
//
// ShareAlike: If you remix, transform, or build upon the material,
// you must distribute your contributions under the same license as the original.
//
// License Deed
// http://creativecommons.org/licenses/by-sa/4.0/
//
// ----------------------------------------------------------------------------------------------------

#include <WiFi.h>



// User define value
// ----------------------------------------------------------------------------------------------------

boolean angle_feed_back_mode = false;
boolean wifi_ap_mode = true; // true = be a WiFi AP; false = connect existing WiFi

// WiFi
const char* ssid = "Q1fab"; // SSID
const char* password = "12345678"; // WiFi password
WiFiServer server(80); // Set web server port number to 80

// ----------------------------------------------------------------------------------------------------



// System value
// ----------------------------------------------------------------------------------------------------

// LED
int ledPin = 13;

// Servo
const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo calibration data
int servoCurrentPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoOldPos[] = { 0, 0, 0, 0, 0, 0, 0, 0}; // Servo old position
int servoPrgPeriod = 30; // 20 ms
int servoPin[] = {18, 17, 16, 4, 23, 22, 21, 19 }; // Initializing servo pin

// Sensor
int sensorPin[] = {35, 26, 12, 33, 34, 25, 27, 32}; // Servo sensor pin
int sensorVal[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Servo sensor analog value

int angle;

// ----------------------------------------------------------------------------------------------------



// Action code
// ----------------------------------------------------------------------------------------------------

// Servo zero position
int servoAct00 [] PROGMEM =
  // S01, S02, S03, S04, S05, S06, S07, S08
  {  135,  45, 135,  45,  45, 135,  45, 135 };

// Zero
int servoPrg00step = 1;
int servoPrg00 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {  135,  45, 135,  45,  45, 135,  45, 135, 1000  }, // zero position
};

// Standby
int servoPrg01step = 2;
int servoPrg01 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // prep standby
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Forward
int servoPrg02step = 11;
int servoPrg02 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  45,  90,  100  }, // leg1,4 up; leg4 fw
  {   70,  90,  90, 110, 110,  90,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90,  90,  90,  90,  45,  70,  100  }, // leg2,3 up
  {   70,  45, 135,  90,  90,  90,  90,  70,  100  }, // leg1,4 bk; leg2 fw
  {   70,  45, 135, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
  {   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg1,4 up; leg1 fw
  {   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg2,3 bk
  {   70,  90,  90, 110, 110, 135,  90,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110,  90, 135,  90,  70,  100  }, // leg3 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg3 fw dn
};

// Backward
int servoPrg03step = 11;
int servoPrg03 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  45,  90, 110, 110,  90,  90,  90,  100  }, // leg4,1 up; leg1 fw
  {   70,  45,  90, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
  {   70,  45,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up
  {   70,  90,  90,  90,  90, 135,  45,  70,  100  }, // leg4,1 bk; leg3 fw
  {   70,  90,  90, 110, 110, 135,  45,  70,  100  }, // leg3,2 dn
  {   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg4,1 up; leg4 fw
  {   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg3,1 bk
  {   70,  90, 135, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
  {   70,  90, 135,  90, 110,  90,  90,  70,  100  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg2 fw dn
};

// Move Left
int servoPrg04step = 11;
int servoPrg04 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  45,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg2 fw
  {   70,  90,  45, 110, 110,  90,  90,  70,  100  }, // leg3,2 dn
  {   90,  90,  45, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  45,  90,  90,  100  }, // leg3,2 bk; leg1 fw
  {   70, 135,  90, 110, 110,  45,  90,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg3 fw
  {   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg1,4 bk
  {   70,  90,  90, 110, 110,  90, 135,  70,  100  }, // leg3,2 dn
  {   70,  90,  90, 110, 110,  90, 135,  90,  100  }, // leg4 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg4 fw dn
};

// Move Right
int servoPrg05step = 11;
int servoPrg05 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  90,  90,  90,  45,  90,  70,  100  }, // leg2,3 up; leg3 fw
  {   70,  90,  90, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
  {   90,  90,  90, 110, 110,  45,  90,  90,  100  }, // leg4,1 up
  {   90,  90,  45, 110, 110,  90, 135,  90,  100  }, // leg2,3 bk; leg4 fw
  {   70,  90,  45, 110, 110,  90, 135,  70,  100  }, // leg4,1 dn
  {   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up; leg2 fw
  {   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg4,1 bk
  {   70, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
  {   90, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg1 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
int servoPrg06 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  90, 135,  90,  100  }, // leg1,4 turn
  {   70, 135,  90, 110, 110,  90, 135,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up
  {   70, 135, 135,  90,  90, 135, 135,  70,  100  }, // leg2,3 turn
  {   70, 135, 135, 110, 110, 135, 135,  70,  100  }, // leg2,3 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Turn right
int servoPrg07step = 8;
int servoPrg07 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  90,  90,  90,  90,  90,  70,  100  }, // leg2,3 up
  {   70,  90,  45,  90,  90,  45,  90,  70,  100  }, // leg2,3 turn
  {   70,  90,  45, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
  {   90,  90,  45, 110, 110,  45,  90,  90,  100  }, // leg1,4 up
  {   90,  45,  45, 110, 110,  45,  45,  90,  100  }, // leg1,4 turn
  {   70,  45,  45, 110, 110,  45,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 1;
int servoPrg08 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,4 up
};

// Say Hi
int servoPrg09step = 4;
int servoPrg09 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
int servoPrg10 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }, // leg1, 2 up ; leg3, 4 down
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }  // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
int servoPrg11 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // start
  {  100,  90,  90,  80,  80,  90,  90, 100,  400  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100,  600  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  700  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100, 1300  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70, 1800  }, // up
  {  135,  90,  90,  45,  45,  90,  90, 135,  200  }, // fast down
  {   70,  90,  90,  45,  60,  90,  90, 135,  500  }, // leg1 up
  {   70,  90,  90,  45, 110,  90,  90, 135,  500  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }  // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
int servoPrg12 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  200  }, // leg1,4 dn
  {   30,  45, 135, 150, 150, 135,  45,  30,  200  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
int servoPrg13 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up; leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
int servoPrg14 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   75,  45, 135, 105, 110, 135,  45,  70,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
int servoPrg15 [][numberOfACE] PROGMEM = {
  // S01, S02, S03, S04, S05, S06, S07, S08,  ms
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // standby
};

// ----------------------------------------------------------------------------------------------------



// Setup
// ----------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Set ledPin to output mode and blink three times
  pinMode(ledPin, OUTPUT);
  startupSignal();

  // set servoPin to output mode
  for (int k = 0; k < numberOfServos; k++) { // Loop for servo
    pinMode(servoPin[k], OUTPUT);
  }

  // set sensorPin to input mode
  for (int s = 0; s < numberOfServos; s++) { // Loop for sensor
    pinMode(sensorPin[s], INPUT);
  }

  if (angle_feed_back_mode == false) {
    // Web server
    if (wifi_ap_mode == true) {
      Serial.println("Configuring access point...");
      WiFi.softAP(ssid, password);
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(myIP);
    } else {
      Serial.print("Connecting to ");
      Serial.println(ssid);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
    server.begin();
  }
}

// ----------------------------------------------------------------------------------------------------



// Loop
// ----------------------------------------------------------------------------------------------------

void loop() {
  Serial.begin(115200);

  if (angle_feed_back_mode == true) {
    // Printout angle of servos
    for (int s = 0; s < numberOfServos; s++) { // Loop for sensor
      sensorVal[s] = analogReadMilliVolts(sensorPin[s]);
      // Serial.print(sensorVal[s]);
      angle = map(sensorVal[s], 128, 3108, 0, 180);
      Serial.print(angle);
      Serial.print(",");
    }
    Serial.println("");
    delay(500);
  }
      // Run something when user change robot leg position
      sensorVal[0] = analogReadMilliVolts(sensorPin[0]);
      sensorVal[4] = analogReadMilliVolts(sensorPin[4]);
      Serial.print(sensorVal[0]);
      Serial.print(",");
      Serial.print(sensorVal[4]);
      Serial.println("");
      if (analogReadMilliVolts(sensorPin[0]) > 2900 && analogReadMilliVolts(sensorPin[4]) < 500) {
        runServoPrg(servoPrg09, servoPrg09step); // sayHi
      }

  if (angle_feed_back_mode == false) {
    // Web server
    WiFiClient client = server.available();   // listen for incoming clients
    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character

            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();

              // the content of the HTTP response follows the header:
              client.print("<html>");
              client.print("<head>");
              client.print("<style>");
              client.print("button {");
              client.print("border: none;");
              client.print("color: white;");
              client.print("padding: 25px 32px;");
              client.print("text-align: center;");
              client.print("font-size: 200%;");
              client.print("border-radius: 10px;");
              client.print("height: 100%;");
              client.print("width: 100%;");
              client.print("}");
              client.print("</style>");
              client.print("</head>");
              client.print("<body style=\"font-family: Arial, Helvetica, sans-serif;font-size: 36px;text-align: center;\">");
              client.print("<div type=\"title\">Q1 fab</div>");
              client.print("<table width=100% height=95%>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #4CAF50;\" onclick=\"window.location.href='/A'\">Turn Left</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #449FD9;\" onclick=\"window.location.href='/B'\">Forward</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #4CAF50;\" onclick=\"window.location.href='/C'\">Turn Right</button></td>");
              client.print("</tr>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #449FD9;\" onclick=\"window.location.href='/D'\">Left</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #449FD9;\" onclick=\"window.location.href='/E'\">Backward</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #449FD9;\" onclick=\"window.location.href='/F'\">Right</button></td>");
              client.print("</tr>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #C53783;\" onclick=\"window.location.href='/G'\">Standby</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/H'\">Say Hi!</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/I'\">Push Up</button></td>");
              client.print("</tr>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/J'\">Lie</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/K'\">Fighting</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/L'\">Sleep</button></td>");
              client.print("</tr>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/M'\">Dancing 1</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/N'\">Dancing 2</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #E79D40;\" onclick=\"window.location.href='/O'\">Dancing 3</button></td>");
              client.print("</tr>");
              client.print("<tr height=16%>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #F093C4;\" onclick=\"window.location.href='/P'\">LED On</button></td>");
              client.print("<td with=30%><button type=\"button\" style=\"background-color: #F093C4;\" onclick=\"window.location.href='/Q'\">LED Off</button></td>");
              client.print("<td with=30%></td>");
              client.print("</tr>");
              client.print("</table>");
              client.print("</body>");
              client.print("</html>");

              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }

          if (currentLine.endsWith("GET /A")) {
              runServoPrg(servoPrg06, servoPrg06step); // turnLeft
          }
          if (currentLine.endsWith("GET /B")) {
              runServoPrg(servoPrg02, servoPrg02step); // forward
          }
          if (currentLine.endsWith("GET /C")) {
              runServoPrg(servoPrg07, servoPrg07step); // turnRight
          }
          if (currentLine.endsWith("GET /D")) {
              runServoPrg(servoPrg04, servoPrg04step); // left
          }
          if (currentLine.endsWith("GET /E")) {
              runServoPrg(servoPrg03, servoPrg03step); // backward
          }
          if (currentLine.endsWith("GET /F")) {
              runServoPrg(servoPrg05, servoPrg05step); // right
          }
          if (currentLine.endsWith("GET /G")) {
              runServoPrg(servoPrg01, servoPrg01step); // standby
          }
          if (currentLine.endsWith("GET /H")) {
              runServoPrg(servoPrg09, servoPrg09step); // sayHi
          }
          if (currentLine.endsWith("GET /I")) {
              runServoPrg(servoPrg11, servoPrg11step); // pushUp
          }
          if (currentLine.endsWith("GET /J")) {
              runServoPrg(servoPrg08, servoPrg08step); // lie
          }
          if (currentLine.endsWith("GET /K")) {
              runServoPrg(servoPrg10, servoPrg10step); // fighting
          }
          if (currentLine.endsWith("GET /L")) {
              runServoPrg(servoPrg12, servoPrg12step); // sleep
          }
          if (currentLine.endsWith("GET /M")) {
              runServoPrg(servoPrg13, servoPrg13step); // dancing1
          }
          if (currentLine.endsWith("GET /N")) {
              runServoPrg(servoPrg14, servoPrg14step); // dancing2
          }
          if (currentLine.endsWith("GET /O")) {
              runServoPrg(servoPrg15, servoPrg15step); // dancing3
          }
          if (currentLine.endsWith("GET /P")) {
            digitalWrite(13, HIGH); // LED on
          }
          if (currentLine.endsWith("GET /Q")) {
            digitalWrite(13, LOW); // LED off
          }
        }
      }
      // close the connection
      client.stop();
      Serial.println("Client Disconnected.");
    }
    // End of Web server
  }

  delay(1000);
}

// ----------------------------------------------------------------------------------------------------



// Function
// ----------------------------------------------------------------------------------------------------

void startupSignal() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(400);
    digitalWrite(ledPin, LOW);
    delay(400);
  }
}

void servoPulse(int servo, int angle) {
  int pwm = (angle*11) + 500; // Convert angle to microseconds
  digitalWrite(servo, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(servo, LOW);
}

void servoPos(int a1, int a2, int a3, int a4, int a5, int a6, int a7, int a8) {
  servoPulse(servoPin[0], a1);
  servoPulse(servoPin[1], a2);
  servoPulse(servoPin[2], a3);
  servoPulse(servoPin[3], a4);
  servoPulse(servoPin[4], a5);
  servoPulse(servoPin[5], a6);
  servoPulse(servoPin[6], a7);
  servoPulse(servoPin[7], a8);
  servoOldPos[0] = a1;
  servoOldPos[1] = a2;
  servoOldPos[2] = a3;
  servoOldPos[3] = a4;
  servoOldPos[4] = a5;
  servoOldPos[5] = a6;
  servoOldPos[6] = a7;
  servoOldPos[7] = a8;
}

void runServoPrg(int servoPrg[][numberOfACE], int step)
{
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoCurrentPos[s] = servoOldPos[s] - servoCal[s];
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servoPulse(servoPin[k], (map(j, 0, totalTime / servoPrgPeriod, servoCurrentPos[k], servoPrg[i][k])) + servoCal[k]);
        servoOldPos[k] = (map(j, 0, totalTime / servoPrgPeriod, servoCurrentPos[k], servoPrg[i][k])) + servoCal[k];
      }
      delay(servoPrgPeriod);
    }
  }
}

// ----------------------------------------------------------------------------------------------------
