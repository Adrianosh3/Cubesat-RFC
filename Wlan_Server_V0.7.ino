//===============================================================
// @file:     GroundControl_V0.1
// @brief:    Communication CubeSat - Ground Control
//
// @authors:  Adrian Setka, Immanuel Weule
//
// @hardware: ESP32-DevKitC V4 (ESP32-WROOM-32U)
// @comments: Can only connect to 2,4 GHz, not to 5 GHz
//
// @date:     2021-07-12
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

#include <ESP32DMASPIMaster.h>

//Set WiFi SSID and password
// const char* ssid = "Kewwin_02"; //WiFi SSID
// const char* password = "2214934027604276"; //WiFi password
const char* ssid = "Apartment 322"; //WiFi SSID
const char* password = "06456469822825645048"; //WiFi password

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
String TMS1 = "50"; 
String TMS2 = "100";
String TMS3 = "200"; 
String TMS4 = "300";
String ODC1, ODC2, ODC3, ODC4, ODC5, ODC6, ODC7;
String PAY1, PAY2, PAY3, PAY4;


int numLog = 0; // numerator for log file
int counter=0;  //Counter for checking connection status in loop

AsyncWebServer server(80);  //Setup a HTTP server



//Just for test purposes of sensormonitoring
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; //I2C sensor connection of BMP280 modul

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE = 8;
const int MCU_Av = 17;  //Set to Pin number, which will be used for MCU Availability
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;

uint8_t spiLength=0;  //First (spi_master_rx_buf[0]) byte of spi message
String spiAddress; //Second (...[1]) and third (...[2]) byte 
uint8_t spiNextPack=0; //Fourth (...[3]) byte
String spiPayload1; //Fifth (...[4]) to sixth (...[5]) byte
String spiPayload2; //Fifth (...[6]) to sixth (...[5]) byte
String spiPayload3; //Fifth (...[8]) to sixth (...[5]) byte
String spiPayload4; //Fifth (...[9]) to sixth (...[5]) byte
uint8_t spiCRC=0; //Last (...[4+spiLength+1]) byte
uint8_t buff;
uint8_t counterSpi=0;
uint8_t transactionNbr=1;

uint8_t asdf=0;

//Change length of array AND mcu_load_size, if mcu status should be tracked over a longer period of time
uint8_t mcu_log[20];
uint8_t mcu_log_size=20;  //Has to be equal to the size of the mcu_log array
String mcu_load=""; //Load of MCU in percent
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
  f.printf("%s%s", ce, ps);
  f.close();
  readFile(SPIFFS, ("/config" + ident + ".txt").c_str());
  return;
}

void sendCommand() {
  const char * commandPt = (readFile(SPIFFS, "/inCommand.txt")).c_str();
  //SPI function to send command
  return;
}

String receiveData(String compareConfig, String data1, String data2, String data3, String data4) {
  // decide wich configuration and update global variables of that module
    if ( conf1 == compareConfig) {
      // EPM
      
      EPM1 = data1;
      EPM2 = data2;
      EPM3 = data3;
      EPM4 = data4;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf1: %s\n", data1, compareConfig, conf1);
      return conf1;
    } else if (conf2 == compareConfig) {
      // ODC
      
      ODC1 = data1;
      ODC2 = data2;
      ODC3 = data3;
      ODC4 = data4;

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
      PAY3 = data3;
      PAY4 = data4;

      printf("\nHat funktioniert.\ndat: %s\ncompareConfig: %s\nconf4: %s\n", data1, compareConfig, conf4);
      return conf4;
    }else{
      printf("\nHat nicht funktioniert.\ndat: %s\ncompareConfig: %s\n", data1, compareConfig);
    }
}

void spi(void){
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

    spiLength=spi_master_rx_buf[0];
    spiAddress=String(spi_master_rx_buf[1]);
    spiNextPack=spi_master_rx_buf[2];

    spiPayload1=String(spi_master_rx_buf[3]);
    spiPayload1+=String(spi_master_rx_buf[4]);
    
    
    spiPayload2=String(spi_master_rx_buf[5]);
    spiPayload2+=String(spi_master_rx_buf[6]);
    
    spiPayload3=String(spi_master_rx_buf[7]);
    spiPayload3+=String(spi_master_rx_buf[8]);
    
    spiPayload4=String(spi_master_rx_buf[9]);
    spiPayload4+=String(spi_master_rx_buf[10]);

    spiCRC=spi_master_rx_buf[2+spiLength+1];

    spiAddress="11";
    //spiPayload="200";

    //switch spiPayload
    switch(asdf){
      case 0:
      //spiAddress="11";
      spiPayload1="50";
      asdf++;
      break;
      case 1:
      //spiAddress="22";
      spiPayload1="100";
      asdf++;
      break;
      case 2:
      //spiAddress="33";
      spiPayload1="150";
      asdf++;
      break;
      case 3:
      //spiAddress="44";
      spiPayload1="200";
      asdf=0;
      break;      
    }

    printf("\nTransaction Nbr: %d", transactionNbr);
    transactionNbr++;
    
    printf("\nReceived:");
    //Show received data (if needed)
    for (uint8_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_master_rx_buf[i]);
    printf("\n");

    printf("Transmitted:");
    //Show transmitted data
    for (uint8_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_master_tx_buf[i]);
    printf("\n");

    printf("Payload: %s\nspiAddress: %s\n", spiPayload1, spiAddress);

    receiveData(spiAddress, spiPayload1, spiPayload2, spiPayload3, spiPayload4);

    switchTxData(); //Only for test purposes to simulate different messages
/*
    //Dispense payload to correct function for further proceeding
    if(spi_master_rx_buf[1]==ce_odc && spi_master_rx_buf[2]==ps_epm)  //epm
    {
      epm();  //tbd
    }else if(spi_master_rx_buf[1]==ce_epm && spi_master_rx_buf[2]==ps_odc)  //odc
    {
      odc();  //tbd
    }else if(spi_master_rx_buf[1]==ce_tms && spi_master_rx_buf[2]==ps_tms)  //tms
    {
      tms();  //tbd
    }else if(spi_master_rx_buf[1]==ce_pay && spi_master_rx_buf[2]==ps_pay)  //pay
    {
      pay();  //tbd
    }else{
      Serial.println("Error, no module with PS %d and ComEn %d connected.", spi_master_rx_buf[1], spi_master_rx_buf[2]);
    }
*/

}

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

  //printf("\n sum:%d mcuLoad: %s\n", mcu_load);    //phne diese Zeile funktioniert der code, mit ihr nicht (nochmal prüfen)

  printf("\n mcuLoad: %s\n", mcu_load);

}

void switchTxData()
{
  switch (counterSpi)
  {
    case 0:
    spi_master_tx_buf[0]=0;
    spi_master_tx_buf[1]=0;
    spi_master_tx_buf[2]=0;
    spi_master_tx_buf[3]=1;
    spi_master_tx_buf[4]=0;
    spi_master_tx_buf[5]=0;
    spi_master_tx_buf[6]=0;
    spi_master_tx_buf[7]=1;
    counterSpi++;
    break;
    case 1:
    spi_master_tx_buf[0]=1;
    spi_master_tx_buf[1]=0;
    spi_master_tx_buf[2]=2;
    spi_master_tx_buf[3]=4;
    spi_master_tx_buf[4]=1;
    spi_master_tx_buf[5]=0;
    spi_master_tx_buf[6]=2;
    spi_master_tx_buf[7]=4;
    counterSpi=0;
    break;
    default:
    spi_master_tx_buf[0]=7;
    spi_master_tx_buf[1]=7;
    spi_master_tx_buf[2]=7;
    spi_master_tx_buf[3]=7;
    spi_master_tx_buf[4]=7;
    spi_master_tx_buf[5]=7;
    spi_master_tx_buf[6]=7;
    spi_master_tx_buf[7]=7;
    counterSpi=0;
    
  }
}

void set_buffer() {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        spi_master_tx_buf[i] = i & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE);
}



//===============================================================
// Test Functions
//===============================================================


String readBMP280Temperature() {
  // Read temperature as Celsius (the default)
  float temp = bmp.readTemperature();
  // Convert temperature to Fahrenheit
  //t = 1.8 * t + 32;
  if (isnan(temp)) {    
    Serial.println("Failed to read temp from BMP280 sensor!");
    return "";
  }
  else {
    File f = SPIFFS.open("/logT.txt", "a");
    time_t t = now();
    f.printf("Value %d : %.2f C   (%d:%d:%d)\n", numLog, temp, hour(t), minute(t), second(t));
    f.close();
    return String(temp);
  }
}

String readBMP280Altitude() {
  float alt = bmp.readAltitude(1013.25);
  if (isnan(alt)) {
    Serial.println("Failed to read alt from BMP280 sensor!");
    return "";
  }
  else {
    File f = SPIFFS.open("/logA.txt", "a");
    time_t t = now();
    f.printf("Value %d : %.2f m   (%d:%d:%d)\n", numLog, alt, hour(t), minute(t), second(t));
    f.close();
    return String(alt);
  }
}

String readBMP280Pressure() {
  float pres = bmp.readPressure() / 100.0F;
  if (isnan(pres)) {
    Serial.println("Failed to read pres from BMP280 sensor!");
    return "";
  }
  else {
    File f = SPIFFS.open("/logP.txt", "a");
    time_t t = now();
    f.printf("Value %d : %.2f hPa   (%d:%d:%d)\n", numLog, pres, hour(t), minute(t), second(t));
    f.close();
    numLog++;
    return String(pres);
  }
}


//===============================================================
// Setup
//===============================================================

void setup(void){
  
  Serial.begin(115200); //Open a serial connection
  Serial.println("Booting...");

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //while (1) delay(10);
  }
  
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

  spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
  spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

  for(int c=0; c<mcu_log_size; c++)
  {
    mcu_log[c]=0;
  }

  set_buffer();
  pinMode(MCU_Av, INPUT);    
  delay(5000);
  master.setDataMode(SPI_MODE3);
  master.setFrequency(1000000);
  master.setMaxTransferSize(BUFFER_SIZE);
  master.setDMAChannel(2);  // 1 or 2 only
  master.setQueueSize(1);   // transaction queue size
    
  master.begin();  // default SPI is HSPI
  
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
    request->send_P(200, "text/plain", mcu_load.c_str());
  });
  
  server.on("/epmValue1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBMP280Temperature().c_str());
  });
  server.on("/epmValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBMP280Altitude().c_str());
  });
  
  server.on("/epmValue3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBMP280Pressure().c_str());
  });
  
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC2.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC3.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC4.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC5.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ODC6.c_str());
  });
  server.on("/odcValue2", HTTP_GET, [](AsyncWebServerRequest *request){
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
  
  //Only use SPI if MCU is available
  if(digitalRead(MCU_Av)==0)  
  {
    spi();
    mcuLoad(0);
  }else{
    mcuLoad(1);
  }
  Serial.println("1");
  //To access your stored values 
  // readFile(SPIFFS, "/configEPM.txt");

  delay(5000);
}
