//===============================================================
// @file:     rfc.ino
// @brief:    Communication CubeSat - Ground Control
//
// @authors:  Adrian Setka, Immanuel Weule
//
// @hardware: ESP32-DevKitC V4 (ESP32-WROOM-32U)
// @comments: Can only connect to 2,4 GHz, not to 5 GHz
//
// @date:     2022-08-06
//
// @links:    git Repo: https://github.com/Adrianosh3/Cubesat-RFC
//
//            used tutorials:
//            https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/
//            https://randomnerdtutorials.com/esp32-async-web-server-espasyncwebserver-library/
//            https://randomnerdtutorials.com/esp32-esp8266-input-data-html-form/
//            https://techtutorialsx.com/2019/06/13/esp8266-spiffs-appending-content-to-file/
//
//            interesting for future features:
//            https://randomnerdtutorials.com/esp32-esp8266-plot-chart-web-server/
//            https://randomnerdtutorials.com/esp32-mpu-6050-web-server/
//===============================================================



//===============================================================
// Header: imports, variable declarations
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
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <ESP32DMASPISlave.h>

//For Wi-Fi hotspot from Laptop
//const char* ssid = "DemoSat"; //WiFi SSID
//const char* password = "123456789"; //WiFi password
//For Wi-Fi from uni
const char* ssid = "DemoSat-WLAN"; //WiFi SSID
const char* password = "24337243"; //WiFi password
//const char* ssid = "Apartment 322"; //WiFi SSID
//const char* password = "06456469822825645048"; //WiFi password

const char* http_username = "admin";  // username for website-login
const char* http_password = "admin";  // password for website-login

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

uint8_t numSPIReceived = 0; // numerator for received SPI messages which are not dummy messages
uint8_t numSPISent = 0; // numerator for sent SPI messages which are not dummy messages
uint8_t numLog = 0; // numerator for log file
uint8_t numError = 0; // numerator for Errors
uint8_t counter=0;  //Counter for checking connection status in loop

AsyncWebServer server(80);  //Setup a HTTP server



//SPI SETTINGS
ESP32DMASPI::Slave slave;   //Opt.: ESP32SPISlave slave (should result in same)

static const uint32_t BUFFER_SIZE = 256;
uint8_t* spi_slave_tx_buf; //Is declared in function "spi(...)" as parameter
uint8_t* spi_slave_rx_buf;

//Variables for data type conversion in correct order             //Conversion sequence data type wise
String spiMessageTx_str;  //Stores received values from website   //String
const char* spiMessageTx_s = "Hello world.";                      //char array (string)
char* spiMessageTx_cp;                                            //char pointer
//char spiMessageTx_c           (declared below)                  //char
//int spiMessageTx_i_dummy[256] (declared below)                  //integer buffer
int spiMessageTx_i[256];                                          //integer
//uint8_t spiMessageTx_ui[256]  (declared below)                  //uint8_t
uint8_t* spiMessageTx=0;  //Final variable for spi()              //uint8_t pointer

int counterMessagesSent=0;  //To trait first message from website differently (booting message)

uint8_t *spiParamMemory=0;  //Remember last sent message

const char* testnachricht = "Hello World."; //For testing purposes of data type conversion only
int spiMessageArrayCounter=0;   //Counts number of entries of website message
int spiMessageThreshhold=60;    //To check if ASCII value is number or letter (<58: Number; >64: Letter)

uint8_t spiLength=0;  //First (spi_slave_rx_buf[0]) byte of SPI message
uint8_t spiCI; //Second (...[1]) byte; 7: NP, 6: ADC-Flag, 5-3: reserved, 2-0: Protocol
String spiAddress=""; //Third (...[2]) byte; 7-4: PS, 3: reserved, 2-0: ComEn
String spiAddressPS="";
String spiAddressComEn="";
uint8_t spiCRC=0;
uint8_t buff;
uint8_t spiTransactionCounter=0; //Counts numbers of spi transactions; makes exception for first message after reset (first spi message to MCU)

int busy = 0;   //?Used only one time in receiveData()? -> Adrian

int rfcbusynotbusy = 0;
     
char spiMessageTx_c[256]= { 0 };
   
int spiMessageTx_i_dummy[256]= { 0 };
    
uint8_t spiMessageTx_ui[256]= { 0 };

//Should be sent to MCU in first transaction
uint8_t startMessage[256] = {0, 3, 83};

//Should be sent if there is no new message to be sent
uint8_t emptyMessage[256] = {0, 1};


//For testing purposes only
uint8_t messageCounter=0;
uint8_t firstMessage[256] = {0, 2};
uint8_t secondMessage[256] = {0, 3};
uint8_t thirdMessage[256] = {0, 4};
uint8_t fourthMessage[256] = {0, 5};

//Only for testing purposes
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

//Declaring pre-defined pins
const int mcuComENPin = 4;  //CS Pin from SPI (IO number, not pin number)

uint8_t t_switch_payload=0;

//Change length of mcu_log array AND mcu_load_size, if mcu status should be tracked over a longer period of time
uint8_t mcu_log[20];
uint8_t mcu_log_size=20;  //Has to be equal to the size of the mcu_log array
String mcu_load=""; //Load of RFC in percent
uint8_t sum=0;

String testData = "";



//===============================================================
// Functions
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

void spi(uint8_t *spiParam){

    //First message after reset should be "startMessage", then parameter passed by spi(...) call
    printf("\nSPI start.\n");
    if(spiTransactionCounter == 0) {  //First transaction is 0. transaction
      spi_slave_tx_buf = startMessage;
    } else {
      spi_slave_tx_buf = spiParam;
    }

/*
    //Print spiMessageTx, for testing purposes only
    Serial.println("\nspiMessageTx: ");
    for(int g=0; g<10; g++)
    {
       printf("%p ", spiMessageTx[g]);
    }
    
    printf("\n");

    //Print spi_slave_tx_buf, for testing purposes only
    Serial.println("\nspi_slave_tx_buf: ");
    for(int g=0; g<10; g++)
    {
       printf("%p ", spi_slave_tx_buf[g]);
    }
*/
    printf("\n");
    //printf("Zuweisung tx_buf abgeschlossen.\n");

    //spiParamMemory = spi_slave_tx_buf;  //Not needed, "spiMessageTx = emptyMessage" should be enough
    spiMessageTx = &emptyMessage[0];  //Reset spiMessageTx, so same message doesn't get sent twice; has to be done, before spi transaction, bc rfc waits for spi transaction    
    
    //Print spiMessageTx, for testing purposes only
    Serial.println("\nspiMessageTx: ");
    for(int g=0; g<10; g++)
    {
       printf("%p ", spiMessageTx[g]);
    }
    
    //Here spi_slave_rx_buf is received and spi_slave_tx_buf is being queued, waiting to be sent
    
    if (slave.remained() == 0)
    {
      slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
      slave.yield();
      printf("\nSlave remained.\n");
    }

    //For testing purposes
    printf("\nTransaction number: %d\n", spiTransactionCounter);
    spiTransactionCounter++;

    //Print sent and received values
    printf("\nReceived:");
    //Show received data
    for (uint16_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_slave_rx_buf[i]);
    printf("\n");

    printf("Transmitted:");
    //Show transmitted data
    for (uint16_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_slave_tx_buf[i]);
    printf("\n");

    printf("\nEnd of SPI transaction.\n");

    //For further processing of received data (depending on module)
    receiveData();
    
}

void receiveData() {
    // decide wich configuration and update global variables of that module

    spiLength=spi_slave_rx_buf[0];                                    // Data package length
    spiCI=spi_slave_rx_buf[1];                                        // Communication Identifier

    spiAddressComEn = String((spi_slave_rx_buf[2] & 0b11110000) >> 4);
    spiAddressPS = String((spi_slave_rx_buf[2] & 0b00001111));
    String compareConfig = spiAddressPS+spiAddressComEn;
    spiCRC = spi_slave_rx_buf[2+spiLength+1];


    if (conf1 == compareConfig) {
      //EPM

      EPM1 = String(spi_slave_rx_buf[3] & 0b00000001);
      EPM2 = String(spi_slave_rx_buf[3] & 0b00000010);
      EPM3 = String(spi_slave_rx_buf[3] & 0b00000100);
      EPM4 = String(spi_slave_rx_buf[3] & 0b00001000);

      printf("\nTransfer to Website.\nconf1: %s\n", conf1);
      //return conf1;
    } else if (conf2 == compareConfig) {
      //ODC

      ODC1 = String(spi_slave_rx_buf[11] << 8 | spi_slave_rx_buf[12]);      // Phototransistors
      ODC2 = String(spi_slave_rx_buf[13] << 8 | spi_slave_rx_buf[14]);      // ^
      ODC3 = String(spi_slave_rx_buf[15] << 8 | spi_slave_rx_buf[16]);      // ^
      ODC4 = String(spi_slave_rx_buf[17] << 8 | spi_slave_rx_buf[18]);      // ^
      ODC5 = String(spi_slave_rx_buf[19] << 8 | spi_slave_rx_buf[20]);      // ^
      ODC6 = String(spi_slave_rx_buf[21] << 8 | spi_slave_rx_buf[22]);      // ^
      ODC7 = String(spi_slave_rx_buf[7] << 24 | spi_slave_rx_buf[8] << 16 | spi_slave_rx_buf[9] << 8 | spi_slave_rx_buf[10]);      // Runtime


      printf("\nTransfer to Website.\nconf2: %s\n", conf2);
      //return conf2;
    } else if (conf3 == compareConfig) {
      //TMS

      TMS1 = String(spi_slave_rx_buf[3] << 8 | spi_slave_rx_buf[4]);
      TMS2 = String(spi_slave_rx_buf[5] << 8 | spi_slave_rx_buf[6]);
      TMS3 = String(spi_slave_rx_buf[7] << 8 | spi_slave_rx_buf[8]);
      TMS4 = String(spi_slave_rx_buf[9] << 8 | spi_slave_rx_buf[10]);

      printf("\nTransfer to Website.\nconf3: %s\n", conf3);
      //return conf3;
    } else if (conf4 == compareConfig) {
      //PAY

      printf("\nNo payload installed.");
      //return conf4;
    } else if (spi_slave_rx_buf[2] == 69){
      // if SPI receive function is an error message compareConfig = "E" = "69"
      File f = SPIFFS.open("/logStatus.txt", "a");
      time_t t = now();
      f.printf("Error %d : Transaction: %d  (%d:%d:%d)\n", numError, spiTransactionCounter, hour(t), minute(t), second(t));
      for(int i = 0; i <= 20; i++)
      {
        f.printf("%d ", spi_slave_rx_buf[i]);
      }
      f.printf("\n\n");
      numError++;
      f.close();
      
      File f2 = SPIFFS.open("/temp.txt", "w");
      for(int i = 0; i <= 20; i++)
      {
        f2.printf("%d ", spi_slave_rx_buf[i]);
      }
      f2.close();

      writeFile(SPIFFS, "/inCommand5.txt", (readFile(SPIFFS, "/inCommand4.txt").c_str()));
      writeFile(SPIFFS, "/inCommand4.txt", (readFile(SPIFFS, "/inCommand3.txt").c_str()));
      writeFile(SPIFFS, "/inCommand3.txt", (readFile(SPIFFS, "/inCommand2.txt").c_str()));
      writeFile(SPIFFS, "/inCommand2.txt", (readFile(SPIFFS, "/inCommand1.txt").c_str()));
      writeFile(SPIFFS, "/inCommand1.txt", (readFile(SPIFFS, "/temp.txt").c_str()));
      
    } else if (!(spiLength == 0 && spiCI == 1)){
      // if SPI receive function is not a dummy message (0 1 0 0 ...)
      File f = SPIFFS.open("/logStatus.txt", "a");
      time_t t = now();
      f.printf("Received SPI message %d : Transaction %d  (%d:%d:%d)\n", numSPIReceived, spiTransactionCounter, hour(t), minute(t), second(t));
      for(int i = 0; i <= 20; i++)
      {
        f.printf("%d ", spi_slave_rx_buf[i]);
      }
      f.printf("\n\n");
      numSPIReceived++;
      f.close();
      
      File f2 = SPIFFS.open("/temp.txt", "w");
      for(int i = 0; i <= 20; i++)
      {
        f2.printf("%d ", spi_slave_rx_buf[i]);
      }
      f2.close();

      writeFile(SPIFFS, "/inCommand5.txt", (readFile(SPIFFS, "/inCommand4.txt").c_str()));
      writeFile(SPIFFS, "/inCommand4.txt", (readFile(SPIFFS, "/inCommand3.txt").c_str()));
      writeFile(SPIFFS, "/inCommand3.txt", (readFile(SPIFFS, "/inCommand2.txt").c_str()));
      writeFile(SPIFFS, "/inCommand2.txt", (readFile(SPIFFS, "/inCommand1.txt").c_str()));
      writeFile(SPIFFS, "/inCommand1.txt", (readFile(SPIFFS, "/temp.txt").c_str()));
    } else {
      printf("\nHat nicht funktioniert.\ncompareConfig: %s\n", compareConfig);
    }

}

//Not working anymore/yet
void mcuLoad(uint8_t actualStatus){
  sum=0;
  //Shift array one byte to the right, so a new value can be added to the array
  for(int c=mcu_log_size; c>0; c--){
    mcu_log[c]=mcu_log[c-1];
  }

  mcu_log[0]=actualStatus;  //First element is most recent status

  for(int c=0; c<mcu_log_size; c++){
    sum+=mcu_log[c];
  }

  mcu_load=String(sum*100/mcu_log_size);

  //printf("\n sum:%d mcuLoad: %s\n", mcu_load);    //ohne diese Zeile funktioniert der code, mit ihr nicht (nochmal prüfen)

  printf("\n mcuLoad: %s\n", mcu_load);
}

//===============================================================
// Setup
//===============================================================

void setup(void){

  
  pinMode(mcuComENPin, OUTPUT);
  digitalWrite(mcuComENPin, HIGH);
  
  delay(10);  

  //Write SPI CS pin low, to signal MCU that RFC is not ready to send/receive messages yet
  digitalWrite(mcuComENPin, LOW);
  
  Serial.begin(115200); //Open a serial connection
  Serial.println("Booting...");
  
  //Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Header for SPIFFS Files
  const char* header1 = "Log File Temperature\n\n";
  const char* header2 = "Log File Altitude\n\n";
  const char* header3 = "Log File Pressure\n\n";
  const char* header4 = "Log File Status\n\n";
  const char* temporary = "temp\n";
   
  writeFile(SPIFFS, "/temp.txt", temporary);
  writeFile(SPIFFS, "/logT.txt", header1);
  writeFile(SPIFFS, "/logA.txt", header2);
  writeFile(SPIFFS, "/logP.txt", header3);
  writeFile(SPIFFS, "/logStatus.txt", header4); 

  writeFile(SPIFFS, "/inCommand1.txt", "Waiting for file system to load...");
  writeFile(SPIFFS, "/inCommand2.txt", "Booting...");
  writeFile(SPIFFS, "/inCommand3.txt", " ");
  writeFile(SPIFFS, "/inCommand4.txt", " ");
  writeFile(SPIFFS, "/inCommand5.txt", " ");

  //Initialize how ESP should act - AP or STA (comment out one initialization)
  //WiFi.mode(WIFI_AP); //Access point mode: stations can connect to the ESP
  WiFi.mode(WIFI_STA); //Station mode: the ESP connects to an access point

  ConnectToWiFi();

  if(!MDNS.begin("cubesat")) {  //Argument of MDNS.begin holds website name (".local" has to be added)
   Serial.println("Error starting mDNS");
   return;
  }

  spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  for(int c=0; c<mcu_log_size; c++)
  {
    mcu_log[c]=0;
  }

  //Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on("/workload", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", mcu_load.c_str());
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

  /* Too much server.on functions, commented out for better runtime; pay functions not needed, since no payload is implemented yet
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
  */

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
  server.on("/downloadStatus", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/logStatus.txt", "text/text", true);
  });

  //Interact with Terminal on Website
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
      spiMessageTx_str = readFile(SPIFFS, ("/inCommand1.txt"));

      File f = SPIFFS.open("/logStatus.txt", "a");
      time_t t = now();
      f.printf("Sent SPI message %d : Transaction %d  (%d:%d:%d)\n", numSPISent, spiTransactionCounter, hour(t), minute(t), second(t));
      f.printf("%s\n\n", spiMessageTx_str.c_str());
      numSPISent++;
      f.close();
      
      //Don't process received data first time, since website sends start message, which has not to be converted
      //if (spiMessageTx_str != "File system ready!") {
      if ( counterMessagesSent > 0) {
        spiMessageTx_s = spiMessageTx_str.c_str();
            
        //For testing of data conversion only
        /*
        Serial.println("spiMessageTx_s");
        Serial.println(spiMessageTx_s);
        Serial.println("testnachricht");
        Serial.println(testnachricht);
        */

        spiMessageTx_cp = strdup(spiMessageTx_s);  //convert const char* to char*

        //For testing of data conversion only
        //Serial.println("spiMessageTx_cp");
        //Serial.println(spiMessageTx_cp);

        //printf("\nStart of conversion.\n");  //For testing of data conversion only
        //printf("First for loop.\n");  //For testing of data conversion only
        
        for(int j=0; j<256; j++){
          spiMessageTx_c[j]=*spiMessageTx_cp++;
          spiMessageTx_i_dummy[j]=(int)spiMessageTx_c[j];
          //printf("%d ", spiMessageTx_i_dummy[j]); //For testing of data conversion only
        }

        //printf("\n\nSecond for loop.\n");  //For testing of data conversion only

        //Conversion only handles entries without "space" and if bytes are seperated with a comma
        for(int j=0; j<256; j++)
        {
          printf("\n[%d] %d ", j, spiMessageTx_i_dummy[j]);
          if((','==spiMessageTx_i_dummy[j]) && spiMessageTx_i_dummy[j] < spiMessageThreshhold)  //If current (j) sign is a comma
          {
            //Do nothing (it's a comma)
          }else if(','==spiMessageTx_i_dummy[j+1] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){ //If j+1 sign is a comma and a single digit number
            spiMessageTx_i[spiMessageArrayCounter]=spiMessageTx_i_dummy[j]-48;
            spiMessageArrayCounter++;
          }else if(','==spiMessageTx_i_dummy[j+1] && spiMessageTx_i_dummy[j] > spiMessageThreshhold){ //If j+1 sign is a comma and a letter
            spiMessageTx_i[spiMessageArrayCounter]=spiMessageTx_i_dummy[j];
            spiMessageArrayCounter++;
          }else if(','==spiMessageTx_i_dummy[j+2] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){ //If j+2 sign is a comma -> double digit
            spiMessageTx_i[spiMessageArrayCounter]=(spiMessageTx_i_dummy[j]-48)*10 + spiMessageTx_i_dummy[j+1]-48;
            spiMessageArrayCounter++;
            j++;
          }else if(','==spiMessageTx_i_dummy[j+3] && spiMessageTx_i_dummy[j] < spiMessageThreshhold){ //If j+2 sign is a comma -> triple digit
            spiMessageTx_i[spiMessageArrayCounter]=(spiMessageTx_i_dummy[j]-48)*100 + (spiMessageTx_i_dummy[j+1]-48)*10 + spiMessageTx_i_dummy[j+2]-48;
            spiMessageArrayCounter++;
            j+=2;
          }else if(NULL==spiMessageTx_i_dummy[j]){
            j=256;
            printf("End of message.");
          }else{
            printf("Error.");
          }
        }
        
        spiMessageArrayCounter=0;
        
        //printf("\nThird for loop.\n");  //For testing of data conversion only
        
        for (int i = 0; i < 256; ++i) {
          spiMessageTx_ui[i] = (int) spiMessageTx_i[i];
        }
        /*
        for(int g=0; g<5; g++)
        {
          printf("%d %d ", g, spiMessageTx_ui[g]);
        }
        */
        spiMessageTx = &spiMessageTx_ui[0];   //Final assignment of final variable for spi transaction

        //Printing converted value, which was received from website (for testing)
        /*
        Serial.println("spiMessageTx: ");
        for(int g=0; g<10; g++)
        {
          printf("%p ", spiMessageTx[g]);
        }
        */
        //printf("\nEnd of conversion.\n");

      }
      counterMessagesSent=1;  //First message (start message from website) has been sent
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

  //spiMessageTx = &emptyMessage[0];
  
  digitalWrite(mcuComENPin, HIGH);  //Write SPI CS pin to high again, to show MCU that RFC is ready to work size

  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(BUFFER_SIZE);
  //slave.setDMAChannel(2);  //1 or 2 only
  slave.setQueueSize(1);   //Transaction queue size
  slave.begin(VSPI);  //Default SPI is HSPI; VSPID: 37, VSPIQ: 31, VSPICLK: 30, VSPICS0: 29

  Serial.println("Setup C-Code fertig.");
  
}



//===============================================================
// Loop
//===============================================================

void loop(void){

  //Check if ESP is still connected to WiFi and reconnect if connection was lost
  if(counter>120) { //Dont check connection status in every loop (runtime)
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected. Try to reconnect...");
      ConnectToWiFi();
    }
    counter=0;
  } else {
    counter++;
  }
  
  spi(spiMessageTx);
  
  //delay(5000);
  
/*
  //Toggle rfc comen pin to simulate rfc busy/not busy; for testing purposes only
  if(rfcbusynotbusy == 0)
  {
    printf("\nLoop:\nRFC busy.\n");
    digitalWrite(mcuComENPin, HIGH);
    rfcbusynotbusy = 1;
  }else{
    printf("\nLoop:\nRFC not busy.\n");
    digitalWrite(mcuComENPin, LOW); 
    rfcbusynotbusy = 0;
  }
  */
/*
  if(messageCounter == 0){
    spiMessageTx=firstMessage;
    messageCounter++;
  }else if(messageCounter == 1){
    messageCounter++;
  }else if(messageCounter == 2){
    spiMessageTx=secondMessage;
    messageCounter++;
  }else if(messageCounter == 3){
    messageCounter++;
  }else if(messageCounter == 4){
    spiMessageTx=thirdMessage;
    messageCounter++;
  }else if(messageCounter == 5){
    messageCounter++;
  }else{
    spiMessageTx=fourthMessage;
    messageCounter=0;
  }
*/
  
  //To access your stored values
  //readFile(SPIFFS, "/configEPM.txt");
}
