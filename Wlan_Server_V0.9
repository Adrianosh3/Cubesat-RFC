//===============================================================
// @file:     GroundControl_V8.0
// @brief:    Communication CubeSat - Ground Control
//
// @authors:  Adrian Setka, Immanuel Weule
//
// @hardware: ESP32-DevKitC V4 (ESP32-WROOM-32U)
// @comments: Can only connect to 2,4 GHz, not to 5 GHz
//
// @date:     2022-01-10
//
// @links:    https://randomnerdtutorials.com/esp32-esp8266-plot-chart-web-server/
//            https://randomnerdtutorials.com/esp32-mpu-6050-web-server/
//            https://randomnerdtutorials.com/esp32-async-web-server-espasyncwebserver-library/
//            https://randomnerdtutorials.com/esp32-esp8266-input-data-html-form/
//
//            https://diyprojects.io/esp8266-web-server-part-1-store-web-interface-spiffs-area-html-css-js/#.YOxFhxOA5eg
//
//            https://techtutorialsx.com/2019/06/13/esp8266-spiffs-appending-content-to-file/
//===============================================================



//===============================================================
// Header files, variable declarations and WiFi setup
//===============================================================

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPIFFS.h> //File system

#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>  //For URL/name instead of IP address
#include <analogWrite.h>
#include <TimeLib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



#include <ESP32DMASPISlave.h>

//Set WiFi SSID and password
//const char* ssid = "5 Euro/min"; //WiFi SSID
//const char* password = "Wir haben kein Passwort!420"; //WiFi password
//const char* ssid = "Apartment 322"; //WiFi SSID
//const char* password = "06456469822825645048"; //WiFi password

//For Wi-Fi hotspot
const char* ssid = "DemoSat"; //WiFi SSID
const char* password = "123456789"; //WiFi password

const char* http_username = "admin";  // username for login
const char* http_password = "admin";  // password for login

const char* PARAM_COMMAND1 = "inCommand1"; //Variable for commandline SPIFFS file
const char* PARAM_COMMAND2 = "inCommand2"; // Variables for last commands
const char* PARAM_COMMAND3 = "inCommand3";
const char* PARAM_COMMAND4 = "inCommand4";
const char* PARAM_COMMAND5 = "inCommand5";
const char* TEST = "test";

//Variables for hardware configuration SPIFFS files
const char* PS_EPM = "PSepm";
const char* PS_ODC = "PSodc";
const char* PS_TMS = "PStms";
const char* PS_PAY = "PSpay";
const char* CE_EPM = "CEepm";
const char* CE_ODC = "CEodc";
const char* CE_TMS = "CEtms";
const char* CE_PAY = "CEpay";
const char* API_EPM = "APIepm";
const char* API_ODC = "APIodc";
const char* API_TMS = "APItms";
const char* API_PAY = "APIpay";

const char* M1on = "M1on";
const char* M1off = "M1off";

//String Array of modules for easier handling
char arr[][4] = {
  "EPM","ODC","TMS","PAY"
};
char arr2[][4] = {
  "epm","odc","tms","pay"
};

String conf1 = "";
String conf2 = "";
String conf3 = "";
String conf4 = "";
String EPM1, EPM2, EPM3, EPM4;
String ODC1, ODC2, ODC3, ODC4, ODC5, ODC6, ODC7;
String PAY1, PAY2, PAY3, PAY4;
String TMS1 = "50";
String TMS2 = "51";
String TMS3 = "52";
String TMS4 = "53";


uint8_t numLog = 0; // numerator for log file
uint8_t counter=0;  //Counter for checking connection status in loop

AsyncWebServer server(80);  //Setup a HTTP server



//SPI SETTINGS
ESP32DMASPI::Slave slave;   //Opt.: ESP32SPISlave slave;

static const uint32_t BUFFER_SIZE = 256;
const int RFC_Av = 17;  //Tells MCU if RFC is available or not
uint8_t* spi_slave_tx_buf; //Is declared in function "spi(...)" as parameter
uint8_t* spi_slave_rx_buf;
uint8_t spiMessageTx_T[256];
const char* spiMessageTx_s = "Hello world.";
char* spiMessageTx_cp;
char spiMessageTx_c[256];
int spiMessageTx_i_dummy[256];
int spiMessageTx_i[256];
int spiMessageArrayCounter=0;
int spiMessageThreshhold=60;    //Threshhold for distinction between numbers and letters
uint8_t* spiMessageTx_uip;
uint8_t spiMessageTx_ui[256];


uint8_t spi_test_counter=0;
uint8_t spi_test_counter2=0;



uint8_t* spiMessageTx=0;
uint8_t spiLength=0;  //First (spi_slave_rx_buf[0]) byte of SPI message
uint8_t spiCI; //Second (...[1]) byte; 7: NP, 6: ADC-Flag, 5-3: reserved, 2-0: Protocol
String spiAddress=""; //Third (...[2]) byte; 7-4: PS, 3: reserved, 2-0: ComEn
String spiAddressPS="";
String spiAddressComEn="";
String spiPayload1; //Fifth (...[4]) to sixth (...[5]) byte
String spiPayload2;
String spiPayload3;
String spiPayload4;
String spiPayload5;
String spiPayload6;
String spiPayload7;
uint8_t spiCRC=0; //Last (...[4+spiLength+1]) byte
uint8_t buff;
uint8_t counterSpi=0;

uint8_t transactionNbr=1;
int busy = 0;

uint8_t t_switch_payload=0;

//Change length of rfc_log array AND rfc_load_size, if rfc status should be tracked over a longer period of time
uint8_t rfc_log[20];
uint8_t rfc_log_size=20;  //Has to be equal to the size of the rfc_log array
String rfc_load=""; //Load of RFC in percent
uint8_t sum=0;

String testData = "";

//===============================================================
// Function declarations
//===============================================================

void ConnectToWiFi() {
  Serial.println("Connecting to ");
  Serial.print(ssid);

  WiFi.disconnect();
  WiFi.begin(ssid, password); //For ESP as a station; for ESP as AP use "WiFi.softAP(ssid, password)"

  //Wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
  }

  //If connection is successful show IP address in serial monitor
  Serial.println("");

  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); //IP address assigned to your ESP

  return;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File f = fs.open(path, "r");
  if(!f || f.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(f.available()){
    fileContent+=String((char)f.read());
  }
  f.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File f = fs.open(path, "w");
  if(!f){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(f.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  f.close();
}

void writeConfig(String ident) {
  String ce = (readFile(SPIFFS, ("/ce" + ident + ".txt").c_str()));
  String ps = (readFile(SPIFFS, ("/ps" + ident + ".txt").c_str()));
  File f = SPIFFS.open("/config" + ident + ".txt", "w");
  f.printf("%s%s", ps, ce);
  f.close();
  readFile(SPIFFS, ("/config" + ident + ".txt").c_str());
  return;
}

void sendCommand() {
  const char * commandPt = (readFile(SPIFFS, "/inCommand.txt")).c_str();
  //SPI function to send command
  return;
}

String receiveData(String compareConfig, String data1, String data2, String data3, String data4, String data5, String data6, String data7) {
  // decide wich configuration and update global variables of that module
    if (conf1 == compareConfig) {
      //EPM

      EPM1 = data1;
      EPM2 = data2;
      EPM3 = data3;
      EPM4 = data4;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf1: %s\n", data1, compareConfig, conf1);
      return conf1;
    } else if (conf2 == compareConfig) {
      //ODC

      ODC1 = data1;
      ODC2 = data2;
      ODC3 = data3;
      ODC4 = data4;
      ODC5 = data5;
      ODC6 = data6;
      ODC7 = data7;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf2: %s\n", data1, compareConfig, conf2);
      return conf2;
    } else if (conf3 == compareConfig) {
      //TMS

      TMS1 = data1;
      TMS2 = data2;
      TMS3 = data3;
      TMS4 = data4;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf3: %s\n", data1, compareConfig, conf3);
      return conf3;
    } else if (conf4 == compareConfig) {
      //PAY

      PAY1 = data1;
      PAY2 = data2;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf4: %s\n", data1, compareConfig, conf4);
      return conf4;
    }else{
      printf("\nHat nicht funktioniert.\ndat: %s\ncompareConfig: %s\n", data1, compareConfig);
    }
    busy = 0;
}

void spi(uint8_t* spi_param){

    //spi_slave_tx_buf = spi_param;
    
    uint8_t spi_param_test[256] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 
    33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 
    49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 
    65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 
    97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 
    113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 
    129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 
    145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 
    161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 
    177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 
    193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 
    209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 
    225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 
    241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256};
    //0 Byte, Length, ASCII "I", Table Pos., Intervall [*100 ms], Len. DP, Data0 - Data3
    uint8_t test_config_epm[256] = 
    {0, 15, 73, 1, 50, 11, 5, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t test_epm[256] = 
    {4, 69, 65, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //Laut Luca
    uint8_t test_rfc_start[256] = 
    {0, 3, 83, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t test_tms[256] = 
    {4, 65, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t test_start[256] = 
    {0, 3, 83, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t test_restart[256] = 
    {0, 3, 82, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t test_dummy[256] = 
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    if(spi_test_counter == 0){
      spi_slave_tx_buf = test_start;
      spi_test_counter = 1;
    }else{
      spi_slave_tx_buf = test_dummy;
      spi_test_counter = 0;
    }

    if(spi_test_counter2 == 0){
      spi_slave_rx_buf = spi_param_test;
      spi_test_counter2 = 1;
    }
    
/*
    if(spi_test_counter == 0){
      spi_slave_tx_buf = test_start;
      spi_test_counter = 1;
    }else if(spi_test_counter == 1){
      spi_slave_tx_buf = test_restart;
      spi_test_counter = 2;
    }else if(spi_test_counter == 2){
      spi_slave_tx_buf = test_epm;
      spi_test_counter = 3;
    }else{
      spi_slave_tx_buf = test_dummy;
    }
*/  
    //spi_slave_rx_buf = spi_param_test;

    if (slave.remained() == 0)
    {
      slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
      slave.yield();
      printf("Slave remained");
    }

    
/*
    //for test purposes
    while (slave.available()) {
        // do something here with received data
      for (uint16_t i = 0; i < 250; ++i)  //i<BUFFER_SIZE
        printf("%d ", spi_slave_rx_buf[i]);
      printf("\n");

      slave.pop();
    }
*/


    spiLength=spi_slave_rx_buf[0];
    spiCI=spi_slave_rx_buf[1];
    //spiAddress=String(spi_slave_rx_buf[2]);


    spiAddressPS=String(spi_slave_rx_buf[0]*1+spi_slave_rx_buf[1]*2+spi_slave_rx_buf[2]*3+spi_slave_rx_buf[3]*4); //TBD: if(spiAddressPS>4): ERROR
    spiAddressComEn=String(spi_slave_rx_buf[5]+spi_slave_rx_buf[6]+spi_slave_rx_buf[7]);  //??? --> Antwort von Mirko
    spiAddress=spiAddressPS+spiAddressComEn;

    spiPayload1=String(spi_slave_rx_buf[3]);
    spiPayload1+=String(spi_slave_rx_buf[4]);

    spiPayload2=String(spi_slave_rx_buf[5]);
    spiPayload2+=String(spi_slave_rx_buf[6]);

    spiPayload3=String(spi_slave_rx_buf[7]);
    spiPayload3+=String(spi_slave_rx_buf[8]);

    spiPayload4=String(spi_slave_rx_buf[9]);
    spiPayload4+=String(spi_slave_rx_buf[10]);

    spiPayload5=String(spi_slave_rx_buf[11]);
    spiPayload5+=String(spi_slave_rx_buf[12]);

    spiPayload6=String(spi_slave_rx_buf[13]);
    spiPayload6+=String(spi_slave_rx_buf[14]);

    spiPayload7=String(spi_slave_rx_buf[15]);
    spiPayload7+=String(spi_slave_rx_buf[16]);

    spiCRC=spi_slave_rx_buf[2+spiLength+1];

    //spiAddress="11";
    //spiPayload="200";

    //switch spiPayload
    switch(t_switch_payload){
      case 0:
      //spiAddress="11";
      spiPayload1="50";
      t_switch_payload++;
      break;
      case 1:
      //spiAddress="22";
      spiPayload1="100";
      t_switch_payload++;
      break;
      case 2:
      //spiAddress="33";
      spiPayload1="150";
      t_switch_payload++;
      break;
      case 3:
      //spiAddress="44";
      spiPayload1="200";
      t_switch_payload=0;
      break;
    }

    printf("\nTransaction Nbr: %d", transactionNbr);
    transactionNbr++;

    printf("\nReceived:");
    //Show received data (if needed)
    for (uint16_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_slave_rx_buf[i]);
    printf("\n");

    printf("Transmitted:");
    //Show transmitted data
    for (uint16_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_slave_tx_buf[i]);
    printf("\n");

    printf("Payload: %s\nspiAddress: %s\n", spiPayload1, spiAddress);

/*

    receiveData(spiAddress, spiPayload1, spiPayload2, spiPayload3, spiPayload4, spiPayload5, spiPayload6, spiPayload7);

*/



/*
    //Dispense payload to correct function for further proceeding
    if(spi_slave_rx_buf[1]==ce_odc && spi_slave_rx_buf[2]==ps_epm)  //epm
    {
      epm();  //tbd
    }else if(spi_slave_rx_buf[1]==ce_epm && spi_slave_rx_buf[2]==ps_odc)  //odc
    {
      odc();  //tbd
    }else if(spi_slave_rx_buf[1]==ce_tms && spi_slave_rx_buf[2]==ps_tms)  //tms
    {
      tms();  //tbd
    }else if(spi_slave_rx_buf[1]==ce_pay && spi_slave_rx_buf[2]==ps_pay)  //pay
    {
      pay();  //tbd
    }else{
      Serial.println("Error, no module with PS %d and ComEn %d connected.", spi_slave_rx_buf[1], spi_slave_rx_buf[2]);
    }
*/

}

void rfcLoad(uint8_t actualStatus){
  sum=0;
  //Shift array one byte to the right, so a new value can be added to the array
  for(int c=rfc_log_size; c>0; c--){
    rfc_log[c]=rfc_log[c-1];
  }

  rfc_log[0]=actualStatus;  //First element is most recent status

  for(int c=0; c<rfc_log_size; c++){
    sum+=rfc_log[c];
  }

  rfc_load=String(sum*100/rfc_log_size);

  //printf("\n sum:%d rfcLoad: %s\n", rfc_load);    //ohne diese Zeile funktioniert der code, mit ihr nicht (nochmal prüfen)

  printf("\n rfcLoad: %s\n", rfc_load);

}

void t_switchTxData()
{
  switch (counterSpi)
  {
    case 0:
    spiMessageTx_T[0]=0;
    spiMessageTx_T[1]=0;
    spiMessageTx_T[2]=0;
    spiMessageTx_T[3]=1;
    spiMessageTx_T[4]=0;
    spiMessageTx_T[5]=0;
    spiMessageTx_T[6]=0;
    spiMessageTx_T[7]=1;
    spiMessageTx_T[8]=0;
    spiMessageTx_T[9]=0;
    spiMessageTx_T[10]=0;
    spiMessageTx_T[11]=1;
    spiMessageTx_T[12]=0;
    spiMessageTx_T[13]=0;
    spiMessageTx_T[14]=0;
    spiMessageTx_T[15]=1;
    counterSpi++;
    break;
    case 1:
    spiMessageTx_T[0]=1;
    spiMessageTx_T[1]=0;
    spiMessageTx_T[2]=2;
    spiMessageTx_T[3]=4;
    spiMessageTx_T[4]=1;
    spiMessageTx_T[5]=0;
    spiMessageTx_T[6]=2;
    spiMessageTx_T[7]=4;
    spiMessageTx_T[8]=1;
    spiMessageTx_T[9]=0;
    spiMessageTx_T[10]=2;
    spiMessageTx_T[11]=4;
    spiMessageTx_T[12]=1;
    spiMessageTx_T[13]=0;
    spiMessageTx_T[14]=2;
    spiMessageTx_T[15]=4;
    counterSpi=0;
    break;
    default:
    spiMessageTx_T[0]=7;
    spiMessageTx_T[1]=7;
    spiMessageTx_T[2]=7;
    spiMessageTx_T[3]=7;
    spiMessageTx_T[4]=7;
    spiMessageTx_T[5]=7;
    spiMessageTx_T[6]=7;
    spiMessageTx_T[7]=7;
    spiMessageTx_T[8]=7;
    spiMessageTx_T[9]=7;
    spiMessageTx_T[10]=7;
    spiMessageTx_T[11]=7;
    spiMessageTx_T[12]=7;
    spiMessageTx_T[13]=7;
    spiMessageTx_T[14]=7;
    spiMessageTx_T[15]=7;
    counterSpi=0;
    }
}

void set_buffer() {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        spi_slave_tx_buf[i] = i & 0xFF;
    }
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}



//===============================================================
// Setup
//===============================================================

void setup(void){

  Serial.begin(115200); //Open a serial connection
  Serial.println("Booting...");

  //Write SPI CS pin low, to signal MCU that RFC is not ready to send/receive messages yet
  delay(1);
  digitalWrite(29, LOW);

  //Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Header for SPIFFS Files
  const char* header1 = "Log File Temperature\n";
  const char* header2 = "Log File Altitude\n";
  const char* header3 = "Log File Pressure\n";
  writeFile(SPIFFS, "/logT.txt", header1);
  writeFile(SPIFFS, "/logA.txt", header2);
  writeFile(SPIFFS, "/logP.txt", header3);

  writeFile(SPIFFS, "/inCommand1.txt", "Booting...");
  writeFile(SPIFFS, "/inCommand2.txt", " ");
  writeFile(SPIFFS, "/inCommand3.txt", " ");
  writeFile(SPIFFS, "/inCommand4.txt", " ");
  writeFile(SPIFFS, "/inCommand5.txt", " ");


  //Initialize how ESP should act - AP or STA (comment out one initialization)
  //WiFi.mode(WIFI_AP); //Access point mode: stations can connect to the ESP
  WiFi.mode(WIFI_STA); //Station mode: the ESP connects to an access point

  ConnectToWiFi();

  spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  for(int c=0; c<rfc_log_size; c++)
  {
    rfc_log[c]=0;
  }

  pinMode(RFC_Av, OUTPUT);
  digitalWrite(RFC_Av, HIGH);
  //delay(5000);  //Is this delay even necessary?
  slave.setDataMode(SPI_MODE3);  //Only 3 works
  slave.setMaxTransferSize(BUFFER_SIZE);
  //slave.setDMAChannel(2);  //1 or 2 only
  slave.setQueueSize(1);   //Transaction queue size

  slave.begin(VSPI);  //Default SPI is HSPI; VSPID: 37, VSPIQ: 31, VSPICLK: 30, VSPICS0: 29

  if(!MDNS.begin("cubesat")) {  //Argument of MDNS.begin holds website name (".local" has to be added)
     Serial.println("Error starting mDNS");
     return;
  }

  //Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on("/workload", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", rfc_load.c_str());
  });

  // EPM
  server.on("/epmValue1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", EPM1.c_str());
  });
  server.on("/epmValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", EPM2.c_str());
  });
  server.on("/epmValue3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", EPM3.c_str());
  });
  server.on("/epmValue4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", EPM4.c_str());
  });


  // ODC
  server.on("/odcValue1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC1.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC2.c_str());
  });
  server.on("/odcValue3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC3.c_str());
  });
  server.on("/odcValue4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC4.c_str());
  });
  server.on("/odcValue5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC5.c_str());
  });
  server.on("/odcValue6", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC6.c_str());
  });
  server.on("/odcValue7", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC7.c_str());
  });


  // Test purpose TMS
  server.on("/tmsValue1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", TMS1.c_str());
  });
  server.on("/tmsValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", TMS2.c_str());
  });
  server.on("/tmsValue3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", TMS3.c_str());
  });
  server.on("/tmsValue4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", TMS4.c_str());
  });


  // PAY
  server.on("/payValue1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", PAY1.c_str());
  });
  server.on("/payValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", PAY2.c_str());
  });
  server.on("/payValue3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", PAY3.c_str());
  });
  server.on("/payValue4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", PAY4.c_str());
  });


  //Commands
  server.on("/command1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/inCommand1.txt", "text/text");
  });
  server.on("/command2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/inCommand2.txt", "text/text");
  });
  server.on("/command3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/inCommand3.txt", "text/text");
  });
  server.on("/command4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/inCommand4.txt", "text/text");
  });
  server.on("/command5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/inCommand5.txt", "text/text");
  });
  
  //Config values
  server.on("/ceepm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ceEPM.txt", "text/text");
  });
  server.on("/psepm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/psEPM.txt", "text/text");
  });
  server.on("/ceodc", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ceODC.txt", "text/text");
  });
  server.on("/psodc", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/psODC.txt", "text/text");
  });
  server.on("/cetms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ceTMS.txt", "text/text");
  });
  server.on("/pstms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/psTMS.txt", "text/text");
  });
  server.on("/cepay", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/cePAY.txt", "text/text");
  });
  server.on("/pspay", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/psPAY.txt", "text/text");
  });
  server.on("/apiepm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/apiEPM.txt", "text/text");
  });
  server.on("/apiodc", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/apiODC.txt", "text/text");
  });
  server.on("/apitms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/apiTMS.txt", "text/text");
  });
  server.on("/apipay", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/apiPAY.txt", "text/text");
  });
  
  //Download log files
  server.on("/download1", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/logT.txt", "text/text", true);
  });
  server.on("/download2", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/logA.txt", "text/text", true);
  });
  server.on("/download3", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/logP.txt", "text/text", true);
  });
  
  //Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    String inputMessage;
    // GET inCommand value on <ESP_IP>/get?inCommand=<inputMessage> and safe SPIFFS file
    if (request->hasParam(PARAM_COMMAND1)) {
      inputMessage = request->getParam(PARAM_COMMAND1)->value();
      writeFile(SPIFFS, "/inCommand5.txt", (readFile(SPIFFS, "/inCommand4.txt").c_str()));
      writeFile(SPIFFS, "/inCommand4.txt", (readFile(SPIFFS, "/inCommand3.txt").c_str()));
      writeFile(SPIFFS, "/inCommand3.txt", (readFile(SPIFFS, "/inCommand2.txt").c_str()));
      writeFile(SPIFFS, "/inCommand2.txt", (readFile(SPIFFS, "/inCommand1.txt").c_str()));
      writeFile(SPIFFS, "/inCommand1.txt", inputMessage.c_str());
      spiMessageTx_s = (readFile(SPIFFS, ("/inCommand1.txt"))).c_str();
      
      spiMessageTx_cp = strdup(spiMessageTx_s);  //convert const char* to char*
      for(int j=0; j<256; j++){
        spiMessageTx_c[j]=*spiMessageTx_cp++;
        spiMessageTx_i_dummy[j]=(int)spiMessageTx_c[j];
        printf("%d ", spiMessageTx_i_dummy[j]);
      }
  
      printf("\n");
  
      for(int j=0; j<256; j++)
      {
        printf("\n[%d] %d ", j, spiMessageTx_i_dummy[j]);
        if((','==spiMessageTx_i_dummy[j]) && spiMessageTx_i_dummy[j] < spiMessageThreshhold)
        {
          //Do nothing (it's a comma)
        }else if(','==spiMessageTx_i_dummy[j+1] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){
          spiMessageTx_i[spiMessageArrayCounter]=spiMessageTx_i_dummy[j]-48;
          spiMessageArrayCounter++;
        }else if(','==spiMessageTx_i_dummy[j+1] && spiMessageTx_i_dummy[j] > spiMessageThreshhold){
          spiMessageTx_i[spiMessageArrayCounter]=spiMessageTx_i_dummy[j];
          spiMessageArrayCounter++;
        }else if(','==spiMessageTx_i_dummy[j+2] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){
          spiMessageTx_i[spiMessageArrayCounter]=(spiMessageTx_i_dummy[j]-48)*10 + spiMessageTx_i_dummy[j+1]-48;
          spiMessageArrayCounter++;
          j++;
        }else if(','==spiMessageTx_i_dummy[j+3] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){
          spiMessageTx_i[spiMessageArrayCounter]=(spiMessageTx_i_dummy[j]-48)*100 + (spiMessageTx_i_dummy[j+1]-48)*10 + spiMessageTx_i_dummy[j+2]-48;
          spiMessageArrayCounter++;
          j+=2;
        }else{
          printf("Error.");
        }
      }
  
      for (int i = 0; i < 256; ++i) {
        spiMessageTx_ui[i] = (int) spiMessageTx_i[i];
      }
      
      spiMessageTx_uip = &spiMessageTx_ui[0];
      
      spiMessageTx=spiMessageTx_uip;  //using ending "[...]_uip" for better overview, but changing for further proceeding
      //spiMessageTx should be final variable
    }
    else if (request->hasParam(TEST)) {
      inputMessage = request->getParam(PARAM_COMMAND1)->value();
      writeFile(SPIFFS, "/inCommand2.txt", inputMessage.c_str());
    }
    //GET inConfig value on <ESP_IP>/get?configForm=<inputMessage> and safe SPIFFS files
    else if (request->hasParam(CE_EPM)) {
      for (int i = 0; i < 4; i++) {

        inputMessage = request->getParam(("CE" + (String)arr2[i]).c_str())->value();
        writeFile(SPIFFS, ("/ce" + (String)arr[i] + ".txt").c_str(), inputMessage.c_str());

        inputMessage = request->getParam(("PS" + (String)arr2[i]).c_str())->value();
        writeFile(SPIFFS, ("/ps" + (String)arr[i] + ".txt").c_str(), inputMessage.c_str());

        inputMessage = request->getParam(("API" + (String)arr2[i]).c_str())->value();
        writeFile(SPIFFS, ("/api" + (String)arr[i] + ".txt").c_str(), inputMessage.c_str());
      }

      for (int i = 0; i < 4; i++) {
        writeConfig(arr[i]);
      }
      conf1 = readFile(SPIFFS, ("/config" + String(arr[0]) + ".txt").c_str());
      conf2 = readFile(SPIFFS, ("/config" + String(arr[1]) + ".txt").c_str());
      conf3 = readFile(SPIFFS, ("/config" + String(arr[2]) + ".txt").c_str());
      conf4 = readFile(SPIFFS, ("/config" + String(arr[3]) + ".txt").c_str());
    }
    else if (request->hasParam(M1on)) {
      String message = "";
      inputMessage = request->getParam(M1on)->value();
      message = conf1 + inputMessage;
      Serial.print(message);
    }
    else if (request->hasParam(M1off)) {
      String message = "";
      inputMessage = request->getParam(M1on)->value();
      message = conf1 + inputMessage;
      Serial.print(message);

    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });


  server.onNotFound(notFound);
  server.begin();

  digitalWrite(29, HIGH);  //Write SPI CS pin to high again, to show MCU that RFC is now ready to send/receive SPI messages
}



//===============================================================
// Loop
//===============================================================

void loop(void){

  //Check if ESP is still connected to WiFi and reconnect if connection was lost
  if(counter>120) { //Dont check connection status in every loop (for better runtime)
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected. Try to reconnect...");
      ConnectToWiFi();
    }
    counter=0;
  } else {
    counter++;
  }
  
  delay(75);
  
  spi(spiMessageTx);

  //To access your stored values
  //readFile(SPIFFS, "/configEPM.txt");

  //Serial.println(*spiMessageTx_s);
  Serial.println("Loop.");
  //Serial.println(*spiMessageTx);
}
