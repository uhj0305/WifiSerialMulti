/*************************************************************
  This sketch implements a simple serial receive terminal
  program for monitoring serial debug messages from another
  board.

  Connect GND to target board GND
  Connect RX line to TX line of target board
  Make sure the target and terminal have the same baud rate
  and serial stettings!

  The sketch works with the ILI9341 TFT 240x320 display and
  the called up libraries.

  The sketch uses the hardware scrolling feature of the
  display. Modification of this sketch may lead to problems
  unless the ILI9341 data sheet has been understood!

  Updated by Bodmer 21/12/16 for TFT_eSPI library:
  https://github.com/Bodmer/TFT_eSPI

  BSD license applies, all text above must be included in any
  redistribution
 *************************************************************/
 //Must use 2.0.13
#include "esp_chip_info.h"
#include <esp_task_wdt.h>
#include <M5Stack.h>
#include <HardwareSerial.h>

#include <M5Stack.h>
#include <SPI.h>
#include <Wire.h>

#include "EEPROM.h"

#if 0
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#endif

#define RELEASE
#define WDT_TIMEOUT 120     // define a 120 seconds WDT (Watch Dog Timer)

#ifdef WEBSERIAL
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#endif

// Libraries for SD card
#include "FS.h"
#include "SD.h"

// Libraries to get time from NTP Server
#include <WiFi.h>
//ssid multi support
#include <WiFiMulti.h>

WiFiMulti wifiMulti;

// Define CS pin for the SD card module
const int chipSelect = 4;

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
String dataMessage;
String MyWifiSsid;
String MyWifiPassword;
time_t now = time (nullptr);
struct tm timeinfo;

// 시리얼 통신 프로토콜 사용될 변수 
String UartRxString="";
String ClientRxString="";

String sendString="";

HardwareSerial MySerial(1);

#define RxPin 16
#define TxPin 17
#define BAUDRATE 115200
#define SER_BUF_SIZE 1024

// The scrolling area must be a integral multiple of TEXT_HEIGHT
#define TEXT_HEIGHT 16  // Height of text to be printed and scrolled
#define TOP_FIXED_AREA \
    16 //14  // Number of lines in top fixed area (lines counted from top of screen)
#define BOT_FIXED_AREA \
    0  // Number of lines in bottom fixed area (lines counted from bottom of
       // screen)
#define YMAX 240  // Bottom of screen area

// The initial y coordinate of the top of the scrolling area
uint16_t yStart = TOP_FIXED_AREA;//TOP_FIXED_AREA; //0; //TOP_FIXED_AREA
// yArea must be a integral multiple of TEXT_HEIGHT
uint16_t yArea = YMAX - TOP_FIXED_AREA - BOT_FIXED_AREA;
// The initial y coordinate of the top of the bottom text line
// uint16_t yDraw = YMAX - BOT_FIXED_AREA - TEXT_HEIGHT;
uint16_t yDraw = YMAX - TEXT_HEIGHT;//0;

// Keep track of the drawing x coordinate
uint16_t xPos = 0;

// For the byte we read from the serial port
char data = 0;

// A few test variables used during debugging
boolean change_colour = 1;
boolean selected      = 1;

// We have to blank the top line each time the display is scrolled, but this
// takes up to 13 milliseconds for a full width line, meanwhile the serial
// buffer may be filling... and overflowing We can speed up scrolling of short
// text lines by just blanking the character we drew
int blank[19];  // We keep all the strings pixel lengths to optimise the speed
                // of the top line blanking

#define MAX_WIFI_AP_LIST 6 //4 ToDo : Add etron's internal wifi ap
const char *ssid_list[] =
{
  "WOO&YOON 5.1",     // must 주차장 홈 와이파이 설정
  "Aaron Woo",        // 아이폰 핫스팟
  "Audi_MMI_8783",    // 이트론
  "ALTIWLAN",         //회사인터넷
  "Public WiFi Free", //공공와이파이
  "wifi",
  NULL
};

const char *password_list[] =
{
  "1234567890",
  "1qaz2wsx",
  "imrm-efzu-srht",
  "Main@#!a",
  NULL, //공공와이파이
  "wifi",
  NULL
};
String f_name;

#define EEPROM_SIZE 1024
#define MAGIC_CODE_VAL 0x5256913
struct config_data{
  unsigned int boot_count;
  bool wifi_ap_mode;
  unsigned int uart_speed;
  unsigned int wifi_multi_offset;
  char wifi_multi_ssid[MAX_WIFI_AP_LIST][33];
  char wifi_multi_password[MAX_WIFI_AP_LIST][65];
  char wifi_ap_ssid[33];
  char wifi_ap_password[65];
  bool tcp_socket_mode;
  unsigned int tcp_server_portnum;
  unsigned int tcp_dest_address;
  unsigned int tcp_dest_portnum;
  bool time_stamp;
  char default_filename[256];
  int uart_cr_lf_mode;
  bool uart_rx_log;
  bool socket_rx_log;
  bool display;
  unsigned int magic_code;
};

struct config_data WiFiSerial_config;

typedef enum {
  enum_magic_code = 0,
  enum_device_pos = sizeof(WiFiSerial_config),
}id_eeprom_address;

id_eeprom_address enum_eeprom_address;

unsigned int ui_uart_speed[8] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}; 

#ifdef WEBSERIAL
AsyncWebServer server(80);
unsigned long last_print_time = millis();
#endif

unsigned int UART_SPEED = 115200;
unsigned int SERVER_PORT = 8000;
WiFiServer server(SERVER_PORT);   

WiFiClient client;
//WiFiClient dest;

String MessageLcd;

bool wifi_mode_ap = false;
int serial_speed = 0;


#include <WebServer.h>
#include <ESPmDNS.h>
////#include <odroid_go.h>
//#include "FS.h"
//#include "SD.h"
#include <WiFiAP.h>
#define LED_BUILTIN 2

const char* ssid = "odroidgo";
const char* password = "odroidgo";

const char* header = "<!DOCTYPE html>"
                     "<html>"
                     "<head>"
                     "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                     "<style>"
                     "  h1 {"
                     "    border-bottom: 1px solid #c0c0c0;"
                     "    margin-bottom: 10px;"
                     "    padding-bottom: 10px;"
                     "    white-space: nowrap;"
                     "  }"
                     "  table {"
                     "    border-collapse: collapse;"
                     "  }"
                     "  th {"
                     "    cursor: pointer;"
                     "  }"
                     "  td.detailsColumn {"
                     "    -webkit-padding-start: 2em;"
                     "    text-align: end;"
                     "    white-space: nowrap;"
                     "  }"
                     "  a.icon {"
                     "    -webkit-padding-start: 1.5em;"
                     "    text-decoration: none;"
                     "  }"
                     "  a.icon:hover {"
                     "    text-decoration: underline;"
                     "  }"
                     "  a.file {"
                     "    background : url(\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAIAAACQkWg2AAAABnRSTlMAAAAAAABupgeRAAABHUlEQVR42o2RMW7DIBiF3498iHRJD5JKHurL+CRVBp+i2T16tTynF2gO0KSb5ZrBBl4HHDBuK/WXACH4eO9/CAAAbdvijzLGNE1TVZXfZuHg6XCAQESAZXbOKaXO57eiKG6ft9PrKQIkCQqFoIiQFBGlFIB5nvM8t9aOX2Nd18oDzjnPgCDpn/BH4zh2XZdlWVmWiUK4IgCBoFMUz9eP6zRN75cLgEQhcmTQIbl72O0f9865qLAAsURAAgKBJKEtgLXWvyjLuFsThCSstb8rBCaAQhDYWgIZ7myM+TUBjDHrHlZcbMYYk34cN0YSLcgS+wL0fe9TXDMbY33fR2AYBvyQ8L0Gk8MwREBrTfKe4TpTzwhArXWi8HI84h/1DfwI5mhxJamFAAAAAElFTkSuQmCC \") left top no-repeat;"
                     "  }"
                     "  a.dir {"
                     "    background : url(\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAd5JREFUeNqMU79rFUEQ/vbuodFEEkzAImBpkUabFP4ldpaJhZXYm/RiZWsv/hkWFglBUyTIgyAIIfgIRjHv3r39MePM7N3LcbxAFvZ2b2bn22/mm3XMjF+HL3YW7q28YSIw8mBKoBihhhgCsoORot9d3/ywg3YowMXwNde/PzGnk2vn6PitrT+/PGeNaecg4+qNY3D43vy16A5wDDd4Aqg/ngmrjl/GoN0U5V1QquHQG3q+TPDVhVwyBffcmQGJmSVfyZk7R3SngI4JKfwDJ2+05zIg8gbiereTZRHhJ5KCMOwDFLjhoBTn2g0ghagfKeIYJDPFyibJVBtTREwq60SpYvh5++PpwatHsxSm9QRLSQpEVSd7/TYJUb49TX7gztpjjEffnoVw66+Ytovs14Yp7HaKmUXeX9rKUoMoLNW3srqI5fWn8JejrVkK0QcrkFLOgS39yoKUQe292WJ1guUHG8K2o8K00oO1BTvXoW4yasclUTgZYJY9aFNfAThX5CZRmczAV52oAPoupHhWRIUUAOoyUIlYVaAa/VbLbyiZUiyFbjQFNwiZQSGl4IDy9sO5Wrty0QLKhdZPxmgGcDo8ejn+c/6eiK9poz15Kw7Dr/vN/z6W7q++091/AQYA5mZ8GYJ9K0AAAAAASUVORK5CYII= \") left top no-repeat;"
                     "  }"
                     "  a.up {"
                     "    background : url(\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAmlJREFUeNpsU0toU0EUPfPysx/tTxuDH9SCWhUDooIbd7oRUUTMouqi2iIoCO6lceHWhegy4EJFinWjrlQUpVm0IIoFpVDEIthm0dpikpf3ZuZ6Z94nrXhhMjM3c8895977BBHB2PznK8WPtDgyWH5q77cPH8PpdXuhpQT4ifR9u5sfJb1bmw6VivahATDrxcRZ2njfoaMv+2j7mLDn93MPiNRMvGbL18L9IpF8h9/TN+EYkMffSiOXJ5+hkD+PdqcLpICWHOHc2CC+LEyA/K+cKQMnlQHJX8wqYG3MAJy88Wa4OLDvEqAEOpJd0LxHIMdHBziowSwVlF8D6QaicK01krw/JynwcKoEwZczewroTvZirlKJs5CqQ5CG8pb57FnJUA0LYCXMX5fibd+p8LWDDemcPZbzQyjvH+Ki1TlIciElA7ghwLKV4kRZstt2sANWRjYTAGzuP2hXZFpJ/GsxgGJ0ox1aoFWsDXyyxqCs26+ydmagFN/rRjymJ1898bzGzmQE0HCZpmk5A0RFIv8Pn0WYPsiu6t/Rsj6PauVTwffTSzGAGZhUG2F06hEc9ibS7OPMNp6ErYFlKavo7MkhmTqCxZ/jwzGA9Hx82H2BZSw1NTN9Gx8ycHkajU/7M+jInsDC7DiaEmo1bNl1AMr9ASFgqVu9MCTIzoGUimXVAnnaN0PdBBDCCYbEtMk6wkpQwIG0sn0PQIUF4GsTwLSIFKNqF6DVrQq+IWVrQDxAYQC/1SsYOI4pOxKZrfifiUSbDUisif7XlpGIPufXd/uvdvZm760M0no1FZcnrzUdjw7au3vu/BVgAFLXeuTxhTXVAAAAAElFTkSuQmCC \") left top no-repeat;"
                     "  }"
                     "  html[dir=rtl] a {"
                     "    background-position-x: right;"
                     "  }"
                     "  #parentDirLinkBox {"
                     "    margin-bottom: 10px;"
                     "    padding-bottom: 10px;"
                     "  }"
                     "  #listingParsingErrorBox {"
                     "    border: 1px solid black;"
                     "    background: #fae691;"
                     "    padding: 10px;"
                     "    display: none;"
                     "  }"
                     "</style>"
                     "<title id=\"title\">Web FileBrowser</title>"
                     "</head>"
                     "<body>";

const char* footer =  "<hr>"
                      "<br>WiFi Serial Monitor by Aaron Woo 2024 uhj0305@gmail.com<br>"
                      "<hr>"
                      "</body>"
                      "</html>";

const char* script = "<script>function sortTable(l){var e=document.getElementById(\"theader\"),n=e.cells[l].dataset.order||\"1\",s=0-(n=parseInt(n,10));e.cells[l].dataset.order=s;var t,a=document.getElementById(\"tbody\"),r=a.rows,d=[];for(t=0;t<r.length;t++)d.push(r[t]);for(d.sort(function(e,t){var a=e.cells[l].dataset.value,r=t.cells[l].dataset.value;return l?(a=parseInt(a,10),(r=parseInt(r,10))<a?s:a<r?n:0):r<a?s:a<r?n:0}),t=0;t<d.length;t++)a.appendChild(d[t])}</script>";
//"<br>Made with &hearts; and &#9749; by ripper121<br>"

String uploadPath = "";
WebServer web_server(80);

/* For auto incremental file name */
char FileName[12] = {0,};

void setup() {
    // Setup the TFT display
    M5.begin();
    M5.Power.begin();
    // M5.Lcd.setRotation(5); // Must be setRotation(0) for this sketch to work
    // correctly
    M5.Lcd.fillScreen(TFT_BLACK);

    // Setup baud rate and draw top banner
    Serial.begin(115200);
    view_boot_message();
#if 0
    esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);                           // add current thread to WDT watch
    esp_task_wdt_reset();                 // reset timer
#else  //ESP32 Lib 3.x defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR == 3
    esp_task_wdt_deinit();  // ensure a watchdog is not already configured
    // v3 board manager detected
    // Create and initialize the watchdog timer(WDT) configuration structure
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,  // Convert seconds to milliseconds
      .idle_core_mask = 1 << 0,          // Monitor core 1 only
      .trigger_panic = true              // Enable panic
    };
    // Initialize the WDT with the configuration structure
    esp_task_wdt_init(&wdt_config);  // Pass the pointer to the configuration structure
    esp_task_wdt_add(NULL);          // Add current thread to WDT watch
    esp_task_wdt_reset();            // reset timer
#endif
    Serial.println("Enable for watchdog...\n\r");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);

    MessageLcd = " Wait for WiFi AP connect... 240903 by Aaron";
    M5.Lcd.drawCentreString(MessageLcd, 320 / 2, 0, 2);
    draw_status(false);
    init_nv();
    WiFiSerial_config.boot_count++;
    update_info();
    delay(2000);
    setup_logger();

    MySerial.setRxBufferSize(SER_BUF_SIZE);
    MySerial.begin(WiFiSerial_config.uart_speed, SERIAL_8N1, RxPin, TxPin);

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
    if(wifi_mode_ap == true) {
      MessageLcd = MyWifiSsid + " : " + WiFi.softAPIP().toString() + ":" + String(SERVER_PORT);//" Serial Terminal : "
    }
    else {
      MessageLcd = WiFi.SSID()+ " : " + WiFi.localIP().toString() + ":" + String(SERVER_PORT);//" Serial Terminal : "
    }
    M5.Lcd.drawCentreString(MessageLcd, 320 / 2, 0, 2);
    draw_status(true);
    // Change colour for scrolling zone text
    if(wifi_mode_ap == true) {
      M5.Lcd.setTextColor(TFT_WHITE,TFT_BLUE);    
    }
    else {
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);  
    }
    // Setup scroll area
    setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);
    //setupScrollArea(0, 0);

    // Zero the array
    for (byte i = 0; i < 18; i++) blank[i] = 0;
  #if 0
  listDir(SD, "/", 0);
  //createDir(SD, "/mydir");
  //listDir(SD, "/", 0);
  //removeDir(SD, "/mydir");
  Serial.println("SD level2!");
  listDir(SD, "/", 2);
  #endif
}

void loop(void) {
  //  These lines change the text colour when the serial buffer is emptied
  //  These are test lines to see if we may be losing characters
  //  Also uncomment the change_colour line below to try them
  //
  //  if (change_colour){
  //  change_colour = 0;
  //  if (selected == 1) {M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK); selected =
  //  0;} else {M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK); selected = 1;}
  //}
  esp_task_wdt_reset();   // Added to repeatedly reset the Watch Dog Timer
  Button_read();
  do_debug();
  uart_handler();
  client_handler();
  web_server.handleClient();
}

void client_handler(){
  if (!client.connected()) {//!server.hasClient()) {
    client = server.available();   // listen for incoming clients
  }
  if (client) {
    while (client.available()) {                           // if there's bytes to read from the client,
      //draw_status(true);
      char client_data = client.read();
      //Send to serial port
      sendString = client_data;

      if((client_data == '\n')||(client_data == '\r')) {
        if(WiFiSerial_config.uart_cr_lf_mode == 1) {
          sendString = "\r\n";
        }
      }
      MySerial.print(sendString);
      MySerial.flush();
      Serial.print(client_data);
      Serial.flush();      
      if(WiFiSerial_config.socket_rx_log == true ) {
        if(WiFiSerial_config.uart_cr_lf_mode == 1) {
          if (client_data == '\n' || client_data == '\r' || xPos > 311) {
              xPos  = 0;
              yDraw = scroll_line();  // It can take 13ms to scroll and blank 16
                                      // pixel lines
          }          
        }
        else {
          if (client_data == '\r' || xPos > 311) {
              xPos  = 0;
              yDraw = scroll_line();  // It can take 13ms to scroll and blank 16
                                      // pixel lines
          }
        }
        if (client_data > 31 && client_data < 128) {
            xPos += M5.Lcd.drawChar(client_data, xPos, yDraw, 2);
            // blank[(18+(yStart-TOP_FIXED_AREA)/TEXT_HEIGHT)%19]=xPos; // Keep
            // a record of line lengths
        }

        if((client_data != '\n') && (client_data != '\r'))
        {
          ClientRxString += client_data;
        }
        // 입력 받은 데이터가 '\n' newline 이면...
        if((client_data == '\n') || (client_data == '\r'))
        { //Logdata write to SD Card
        if(ClientRxString.length() > 0)
          save_client_data(ClientRxString);
          ClientRxString = "";
        }      
        //draw_status(false);
      }
    }
  }
}

void uart_handler()
{
  while (MySerial.available()) {
      esp_task_wdt_reset();   // Added to repeatedly reset the Watch Dog Timer
      data = MySerial.read();
      #if ECHO_ON
      if(client.connected() == true) {
        client.print(data);
      }
      #endif
   
      if(WiFiSerial_config.uart_rx_log == true ) {
        //draw_status(true);
        //Serial.println("Data Read!");
        // If it is a CR or we are near end of line then scroll one line
        if(WiFiSerial_config.uart_cr_lf_mode == 1) {
          if (data == '\n' || data == '\r' || xPos > 311) {
              xPos  = 0;
              yDraw = scroll_line();  // It can take 13ms to scroll and blank 16
                                      // pixel lines
          }          
        }
        else {
          if (data == '\r' || xPos > 311) {
              xPos  = 0;
              yDraw = scroll_line();  // It can take 13ms to scroll and blank 16
                                      // pixel lines
          }
        }
        
        if (data > 31 && data < 128) {
            xPos += M5.Lcd.drawChar(data, xPos, yDraw, 2);
            // blank[(18+(yStart-TOP_FIXED_AREA)/TEXT_HEIGHT)%19]=xPos; // Keep
            // a record of line lengths
        }
        // change_colour = 1; // Line to indicate buffer is being emptied        
        // 변수에 추가함.
        if((data != '\n') && (data != '\r'))
        {
          UartRxString += data;
        }
        // 입력 받은 데이터가 '\n' newline 이면...
        if((data == '\n') || (data == '\r'))
        { //Logdata write to SD Card
        if(UartRxString.length() > 0)
          save_serial_data(UartRxString);
          UartRxString = "";
        }
        //draw_status(false);
      }  
  }
}

// ##############################################################################################
// Call this function to scroll the display one text line
// ##############################################################################################
bool first_page_scroll = false;
int scroll_clear_bat_y = TEXT_HEIGHT;
int scroll_line() {
    int yTemp = yStart;  // Store the old yStart, this is where we draw the next line
    // Use the record of line lengths to optimise the rectangle size we need to
    // erase the top line
    //M5.Lcd.fillRect(0,yStart,blank[(yStart-TOP_FIXED_AREA)/TEXT_HEIGHT],TEXT_HEIGHT,TFT_BLACK);
  
    // Change the top of the scroll area
    yStart += TEXT_HEIGHT; //TOP_FIXED_AREA
    // The value must wrap around as the screen memory is a circular buffer
    //if (yStart >= YMAX - BOT_FIXED_AREA) 
    //yStart = TOP_FIXED_AREA + (yStart - YMAX + BOT_FIXED_AREA);
    
    if (yStart >= YMAX) {
      yStart = TOP_FIXED_AREA;
    }

    // Now we can scroll the display

    scrollAddress(yStart);

    if(wifi_mode_ap == true) {
      if(first_page_scroll == false) {
        M5.Lcd.fillRect(0, scroll_clear_bat_y, 320, TEXT_HEIGHT, TFT_BLUE);//TFT_BLACK);//YMAX- TEXT_HEIGHT
      }
      else {
        M5.Lcd.fillRect(0, scroll_clear_bat_y, 320, TEXT_HEIGHT, TFT_BLUE);//TFT_BLACK);//YMAX - TEXT_HEIGHT
      }
      scroll_clear_bat_y += TEXT_HEIGHT;
      if(scroll_clear_bat_y  >= (YMAX) ) {
        scroll_clear_bat_y = TEXT_HEIGHT;
      }      
    }
    else {
      //printf("\n\rM5.Lcd.fillRect yStart = %d, TEXT_HEIGHT = %d, yStart - TEXT_HEIGHT = %d, scroll_clear_bat_y = %d \n\r", yStart, TEXT_HEIGHT, yStart - TEXT_HEIGHT, scroll_clear_bat_y);
      if(first_page_scroll == false) {
        M5.Lcd.fillRect(0, scroll_clear_bat_y, 320, TEXT_HEIGHT, TFT_BLACK);//TFT_BLACK);//YMAX- TEXT_HEIGHT
      }
      else {
        M5.Lcd.fillRect(0, scroll_clear_bat_y, 320, TEXT_HEIGHT, TFT_BLACK);//TFT_BLACK);//YMAX - TEXT_HEIGHT
      }
      scroll_clear_bat_y += TEXT_HEIGHT;
      if(scroll_clear_bat_y  >= (YMAX) ) {
        scroll_clear_bat_y = TEXT_HEIGHT;
        //M5Screen2bmp(SD,"/scroll.bmp");
      }
    }
    first_page_scroll = true;
    return yTemp;
}

// ##############################################################################################
// Setup a portion of the screen for vertical scrolling
// ##############################################################################################
// We are using a hardware feature of the display, so we can only scroll in
// portrait orientation
void setupScrollArea(uint16_t tfa, uint16_t bfa) {
    M5.Lcd.writecommand(ILI9341_VSCRDEF);  // Vertical scroll definition
    M5.Lcd.writedata(tfa >> 8);            // Top Fixed Area line count
    M5.Lcd.writedata(tfa);
    M5.Lcd.writedata((YMAX - tfa - bfa) >>
                     8);  // Vertical Scrolling Area line count
    M5.Lcd.writedata(YMAX - tfa - bfa);
    M5.Lcd.writedata(bfa >> 8);  // Bottom Fixed Area line count
    M5.Lcd.writedata(bfa);
}

// ##############################################################################################
// Setup the vertical scrolling start address pointer
// ##############################################################################################
void scrollAddress(uint16_t vsp) {
    M5.Lcd.writecommand(ILI9341_VSCRSADD);  // Vertical scrolling pointer
    M5.Lcd.writedata(vsp >> 8);
    M5.Lcd.writedata(vsp);
}

void setup_logger()
{
  
  if(WiFiSerial_config.wifi_ap_mode == true) {
    wifi_mode_ap = true;
    mode_chage_to_ap();
  }
  else {
    wifi_mode_ap = false;
    mode_chage_to_sta();
  }  
  
  //mode_chage_to_ap();
#ifdef WEBSERIAL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access webserial interface at http://" + WiFi.localIP().toString() + "/webserial");
  });

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  /* Attach Message Callback */
  WebSerial.onMessage([&](uint8_t *data, size_t len) {
    Serial.printf("Received %u bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i=0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
  });
#endif
  if(wifi_mode_ap == false) {
    if(WiFiSerial_config.time_stamp == true)
    {
      setClock();
    }
    String str_email;

    str_email = "Test Message send here!";
    send_mail(str_email);
  }
  else
  {
    //f_name = "/Logfile.txt";
    Serial.println(f_name);
  }

  init_sd_card();
  draw_status(true);
/*
  Serial.print("Initializing SD card...");
 
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open(f_name.c_str());//"/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    if(wifi_mode_ap == false) {
      writeFile(SD, f_name.c_str() , "Date, Time, Serial Data Logging \r\n");
    }
    else
    {
      writeFile(SD, f_name.c_str() , "Serial Data Logging \r\n");
    }
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
*/  
 }

 void save_client_data(String Logdata)
{
  if(wifi_mode_ap == false) {
    if(WiFiSerial_config.time_stamp == true) {
      getTimeStamp();
      dataMessage = timeStamp + ":" + Logdata + "\n\r";
    }
    else {
      dataMessage = Logdata + "\n\r";
    }
  }
  else
  {
    dataMessage = Logdata + "\n\r";
  }

#ifndef RELEASE  
  Serial.print("Save data: ");
  Serial.print(dataMessage);
#endif  
  //test appendFile(SD, f_name.c_str(), dataMessage.c_str());
  appendFile(SD, f_name.c_str(), dataMessage.c_str());
  //delay(10);                    //Take samples every one second
}
 
void save_serial_data(String Logdata)
{
  if(wifi_mode_ap == false) {
    if(WiFiSerial_config.time_stamp == true) {
      getTimeStamp();
      dataMessage = timeStamp + ":" + Logdata + "\n\r";
    }
    else {
      dataMessage = Logdata + "\n\r";
    }
  }
  else
  {
    dataMessage = Logdata + "\n\r";
  }

#ifndef RELEASE  
  Serial.print("Save data: ");
  Serial.print(dataMessage);
#endif  
  if (client.connected()) {
  //if(client) {
    client.print(dataMessage); 
    client.flush();
  }

  //test appendFile(SD, f_name.c_str(), dataMessage.c_str());
  appendFile(SD, f_name.c_str(), dataMessage.c_str());
  //delay(10);                    //Take samples every one second
}

// Function to get date and time from NTP
char strftime_buf[64];
void getTimeStamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;  
  localtime_r (&now, &timeinfo);
  //Serial.print ("\n");
  //Serial.print ("Current time : ");
  //Serial.print (asctime (&timeinfo));  
  //timeStamp = asctime(&timeinfo); 
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  timeStamp = String(strftime_buf);
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
int print_cr = 0;
#define page_cr 40
void appendFile(fs::FS &fs, const char * path, const char * message) {
  #ifdef RELEASE
    //Serial.print(":");
  #else
    Serial.printf("Appending to file: %s\n", path);
  #endif

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    #ifdef RELEASE
      Serial.print("F");
    #else
      Serial.println("Failed to open file for appending");
    #endif
    return;
  }
  if(file.print(message)) {
    #ifdef RELEASE
      Serial.print("O");
    #else
      Serial.print("Message appended");
    #endif
  } else {
    #ifdef RELEASE
       Serial.print("A");
    #else
      Serial.print("Append failed");
    #endif    
  }
  print_cr++;
  if(print_cr%page_cr == 0){
    Serial.println();
  }  
  file.close();
}

// Set time via NTP, as required for x.509 validation
void setClock() {
    configTime (3600*9, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print ("Waiting for NTP time sync : ");
    time_t now = time (nullptr);
    while (now < 8 * 3600 * 2) {
        delay (500);
        Serial.print (".");
        now = time (nullptr);
    }
    struct tm timeinfo;
    String f_month;
    String f_day;
    gmtime_r (&now, &timeinfo);
    Serial.print ("\n");
    Serial.print ("Current time : ");
    Serial.print (asctime (&timeinfo));
    if(timeinfo.tm_mon < 9) { //10월보다 작으면 '0'추가
      f_month = "0" + String(timeinfo.tm_mon + 1);
    }
    else {
      f_month =  String(timeinfo.tm_mon + 1);
    }    
    if(timeinfo.tm_mday < 10) { //10월보다 작으면 '0'추가
      f_day = "0" + String(timeinfo.tm_mday);
    }
    else {
      f_day =  String(timeinfo.tm_mday);
    }
    //test f_name = "/" + String(timeinfo.tm_year+1900) + f_month + f_day + ".txt";
    f_name = "/" + String(FileName);
    Serial.println(f_name);
}

void display_info()
{
  scrollAddress(0);

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
  if(wifi_mode_ap == true) {
    MessageLcd =  MyWifiSsid + " : " + WiFi.softAPIP().toString() + ":" + String(SERVER_PORT);
  }
  else {
    MessageLcd =  WiFi.SSID() + " : " + WiFi.localIP().toString() + ":" + String(SERVER_PORT);
  }
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.drawCentreString(MessageLcd, 320 / 2, 0, 2);
  draw_status(true);
  // Change colour for scrolling zone text
  if(wifi_mode_ap == true) {
    M5.Lcd.setTextColor(TFT_WHITE,TFT_BLUE);    
      //M5Screen2bmp(SD,"/con_ap.bmp");
  }
  else {
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);  
      //M5Screen2bmp(SD,"/con_sta.bmp");
  }
}


int counter;
char fn[100];
void Button_read() {
  M5.update();  // Read the press state of the key.
  if(M5.BtnA.isPressed()){
    sprintf(fn, "/Counter%d.bmp", counter);
    //M5Screen2bmp(SD,fn);
  }
    if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000, 200)) {
        //M5.Lcd.print('A');
        //display_info();
        MySerial.end();
        serial_speed++;
        //M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
            
            //M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
            //M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
            //drawCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color)
                    
        M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
        
        switch(serial_speed) {
          case 1:
            MySerial.begin(9600, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 9600bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/9600.bmp");
            WiFiSerial_config.uart_speed = 9600;
            update_info();            
            delay(500);
            break;
          case 2:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            //M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
            //M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);          
            MySerial.begin(19200, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 19200bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/19200.bmp");
            WiFiSerial_config.uart_speed = 19200;
            update_info();            
            delay(500);
            break;
          case 3:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            MySerial.begin(38400, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 38400bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/38400.bmp");
            WiFiSerial_config.uart_speed = 38400;
            update_info();            
            delay(500);
            break;
          case 4:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            MySerial.begin(57600, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 57600bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/57600.bmp");
            WiFiSerial_config.uart_speed = 57600;
            update_info();            
            delay(500);
            break;
          case 5:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            //M5.Lcd.setTextDatum(TC_DATUM);
            MySerial.begin(115200, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 115200bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/115200.bmp");
            WiFiSerial_config.uart_speed = 115200;
            update_info();            
            delay(500);
            //serial_speed = 0;
            break;       
          case 6:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            //M5.Lcd.setTextDatum(TC_DATUM);
            MySerial.begin(230400, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 230400bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/230400.bmp");
            WiFiSerial_config.uart_speed = 230400;
            update_info();            
            delay(500);
            //serial_speed = 0;
            break;                                           
          case 7:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            //M5.Lcd.setTextDatum(TC_DATUM);
            MySerial.begin(460800, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 460800bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/460800.bmp");
            WiFiSerial_config.uart_speed = 460800;
            update_info();            
            delay(500);
            //serial_speed = 0;
            break;   
          case 8:
            M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
            //M5.Lcd.setTextDatum(TC_DATUM);
            MySerial.begin(921600, SERIAL_8N1, RxPin, TxPin);
            M5.Lcd.drawCentreString("[[[[ Serial Speed :: 921600bps ]]]]", 320 / 2, 0, 2);
            //M5Screen2bmp(SD,"/921600.bmp");
            WiFiSerial_config.uart_speed = 921600;
            update_info();            
            delay(500);
            serial_speed = 0;
            break;             
        }
        //if(serial_speed == 5) {
        //  serial_speed = 0;
        //}
    } else if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000, 200)) {
        M5.Lcd.print('B');
        //file_send();
        #if 1
        if(wifi_mode_ap == true) {
          printf("Mode STA configuration!!!.\n\r");        
          wifi_mode_ap = false;
          WiFiSerial_config.wifi_ap_mode = false;
          update_info();            
          mode_chage_to_sta();
          Serial.flush();
          myshutdown();
        }
        else {
          printf("Mode AP configuration!!!.\n\r");       
          wifi_mode_ap = true;
          WiFiSerial_config.wifi_ap_mode = true;
          update_info();
          mode_chage_to_ap();
          Serial.flush();
          myshutdown();
        }
        #endif
    } //else if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000, 200)) {
}

void mode_chage_to_ap()
{
  //mode chagne to AP mode!!!
  M5.Lcd.clear(TFT_BLUE);  // Clear the screen and set white to the
                        // background color.
  //M5.Lcd.setCursor(0, 0);
  wifi_mode_ap = true;
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLUE);  
  M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
  MessageLcd = " Wait for WiFi AP connect... 240903 by Aaron";
  M5.Lcd.drawCentreString(MessageLcd, 320 / 2, 0, 2);  

  if (client.connected()) {
    client.stop();
    delay(250);
    server.stop();
    delay(250);
    Serial.println("Client Disconnected.");
  }
  //M5Screen2bmp(SD,"/wait_ap.bmp");  
  //wifi connected? > disconnect wifi > wifi mode change to ap mode > server start
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    delay(250);
  }
  WiFi.mode(WIFI_AP);
  delay(250);
  MyWifiSsid = String(WiFiSerial_config.wifi_ap_ssid);
  MyWifiPassword = String(WiFiSerial_config.wifi_ap_password);
  //Serial.printf("WiFi.mode(WIFI_AP) MyWifiPassword =  %s length = %d\n\r", MyWifiPassword, MyWifiPassword.length() );
  //Serial.printf("WiFi.mode(WIFI_AP) ssid = %s password =  %s length = %d\n\r",WiFiSerial_config.wifi_ap_ssid, WiFiSerial_config.wifi_ap_password, strlen(WiFiSerial_config.wifi_ap_password) );
  IPAddress ip(192, 168, 4, 1);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  delay(250);  
  WiFi.softAP(MyWifiSsid.c_str(), MyWifiPassword.c_str());
  delay(250);

  //WiFi.softAP("Test_Wifi", "1234567890");//WiFiSerial_config.wifi_ap_password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  delay(500);
  printf("Server begin port number = %d.\n\r", SERVER_PORT);
  server.begin(SERVER_PORT);        
  delay(500);
  display_info();
}

void get_wifi_default_ap_ssid()
{
  uint64_t chipid;
  String strUcase;
  MyWifiPassword = "1234567890";
  chipid=ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes)
  Serial.printf("\n\rESP32 Chip ID = %04X", (uint16_t)(chipid>>32)); // print High 2 bytes
  Serial.printf("%08X\n\r", (uint32_t)chipid); // print Low 4bytes        
  strUcase = String((uint32_t)chipid, HEX);
  strUcase.toUpperCase();
  MyWifiSsid = "WiFiSerial_" + strUcase; //Last 4bytes:hex use
}

void mode_chage_to_sta()
{
  wifi_mode_ap = false;
  M5.Lcd.clear(TFT_BLACK);  // Clear the screen and set white to the
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);

  MessageLcd = " Wait for WiFi STA connect... 240903 by Aaron";
  M5.Lcd.drawCentreString(MessageLcd, 320 / 2, 0, 2);      
  draw_status(false);
  if (client.connected()) {
    client.stop();
    delay(250);
    server.stop();
    delay(250);
    Serial.println("Client Disconnected.");
  }
  //M5Screen2bmp(SD,"/wait_sta.bmp");
  //wifi connected? > disconnect wifi > wifi mode change to ap mode > server start
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    delay(250);
  }
  // Connect to Wi-Fi network with SSID and password
  Serial.println();
  Serial.printf("[WiFi] Connecting to Multi WiFi ");
  WiFi.mode(WIFI_STA);
  delay(250);
  /*
      Add your home, office APs here, as many access points as you like
      ESP32 will use the available access point with the strognest signal.
  */
  for(int i=0; i<MAX_WIFI_AP_LIST; i++) {
    wifiMulti.addAP(ssid_list[i], password_list[i]);
    Serial.printf("[%-33s] ", ssid_list[i]);
  }
  Serial.println();

  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  String MkMsg = "WiFi " + WiFi.SSID() + " ip address " + WiFi.localIP().toString() + ":" + String(SERVER_PORT) + " Connected.";
  Serial.println(MkMsg);
  if(WiFiSerial_config.time_stamp == true)
  {
    setClock();
  }
  display_info();

  if(WiFiSerial_config.tcp_socket_mode == true) { //server
    printf("Server begin port number = %d.\n\r", SERVER_PORT);
    server.begin(SERVER_PORT);
    //webserver();
    //webserver_setup();
#ifdef FEATURE_USE_WEB_SERIAL    
    web_serial_setup();
#endif    
  }
  else {//client
    IPAddress _ip;
    _ip = WiFiSerial_config.tcp_dest_address;
    if (!client.connect(_ip, WiFiSerial_config.tcp_dest_portnum)) {
      Serial.println("Connection failed.");
      Serial.println("Waiting 5 seconds before retrying...");
      delay(5000);
      return;
    }
    else {
      Serial.print("TCP socket server connect OK!\n\rip address=");
      Serial.print(_ip);
      Serial.print(", port=");
      Serial.println(WiFiSerial_config.tcp_dest_portnum);      
    }
  }
}

/*
  unsigned int boot_count;
  bool wifi_ap_mode;
  unsigned int uart_speed;
  unsigned int wifi_multi_offset;
  char wifi_multi_ssid[MAX_WIFI_AP_LIST][33];
  char wifi_multi_password[MAX_WIFI_AP_LIST][65];
  unsigned int magic_code;
*/
void update_info()
{
  WiFiSerial_config.wifi_ap_mode = wifi_mode_ap;
  EEPROM.put(enum_magic_code, WiFiSerial_config);
  EEPROM.commit();
  printf("NV Item Write OK...\n\r");
  EEPROM.get(enum_magic_code, WiFiSerial_config);
  display_serialwifi_info();
}

void clear_nv()
{
  for(int i=0; i<EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
 
  WiFiSerial_config.magic_code = MAGIC_CODE_VAL;
  WiFiSerial_config.boot_count = 0;
  WiFiSerial_config.uart_cr_lf_mode = 0;
  wifi_mode_ap = true;
  WiFiSerial_config.wifi_ap_mode = true; //wifi_mode_ap
  WiFiSerial_config.uart_speed = ui_uart_speed[4]; //115200bps default
  WiFiSerial_config.wifi_multi_offset = MAX_WIFI_AP_LIST;
  WiFiSerial_config.tcp_server_portnum = 8000;
  WiFiSerial_config.time_stamp = true;
  WiFiSerial_config.uart_rx_log = true;
  WiFiSerial_config.socket_rx_log = true;
  WiFiSerial_config.display = true;
  f_name = "/Logfile.txt";
  strcpy(WiFiSerial_config.default_filename, f_name.c_str());

  for(int i=0; i<MAX_WIFI_AP_LIST; i++) {
    String ssid_str;
    String password_str;
    ssid_str = ssid_list[i];
    password_str = password_list[i];
    ssid_str.toCharArray(WiFiSerial_config.wifi_multi_ssid[i], sizeof(WiFiSerial_config.wifi_multi_ssid[i]));
    password_str.toCharArray(WiFiSerial_config.wifi_multi_password[i], sizeof(WiFiSerial_config.wifi_multi_password[i]));
  }

  WiFiSerial_config.tcp_dest_address = 16820416;//(uint32_t)dest_ip_addr; 192.168.0.1
  WiFiSerial_config.tcp_dest_portnum = 8000;

  get_wifi_default_ap_ssid();
  
  strcpy(WiFiSerial_config.wifi_ap_ssid, MyWifiSsid.c_str());
  strcpy(WiFiSerial_config.wifi_ap_password, MyWifiPassword.c_str());
  
  EEPROM.put(enum_magic_code, WiFiSerial_config);
  EEPROM.commit();
  EEPROM.get(enum_magic_code, WiFiSerial_config);
  printf("Clear NV Memory...\n\r");
}

void init_nv()
{
  //Init EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(enum_magic_code, WiFiSerial_config);
  if(WiFiSerial_config.magic_code != MAGIC_CODE_VAL) { //Position Error == Clear EEPROM Aerea
    clear_nv();
  }else 
  {
  }
  wifi_mode_ap = WiFiSerial_config.wifi_ap_mode;
  SERVER_PORT = WiFiSerial_config.tcp_server_portnum;
  UART_SPEED = WiFiSerial_config.uart_speed;
  f_name = String(WiFiSerial_config.default_filename);  
  
  printf("NV Memory Read OK...\n\r");
  printf("\n\rDevice power on boot count = %d\n\r\n\r", WiFiSerial_config.boot_count);
  //display_serialwifi_info();
  //update_info();
}

void view_boot_message()
{
  char tempbuf[50];

  printf("\n\r****************************************");
  printf("\n\r*       WiFi Serial Monitor V1.0       *");
  printf("\n\r*         (Altimobility Since 2023)    *");
  printf("\n\r****************************************");
  printf("\n\r*                                      *");
  sprintf(tempbuf,"\n\r*  Compiled : %s %s     *",__DATE__,__TIME__);
  printf(tempbuf);
  printf("\n\r*  Copyright(c) 2023 Woo Hyuk Joon     *");
  printf("\n\r*                                      *");
  printf("\n\r*  Phone : +82-10-9925-6913            *");
  printf("\n\r*  Mail : uhj0305@gmail.com            *");
  //printf("\n\r*                                      *");
  printf("\n\r*                                      *");
  printf("\n\r****************************************\n\r");
}

void display_serialwifi_info()
{
  IPAddress client_destination_ip;
  //view_boot_message();
/*
struct config_data{
  unsigned int boot_count;
  bool wifi_ap_mode;
  unsigned int uart_speed;
  unsigned int wifi_multi_offset;
  char wifi_multi_ssid[MAX_WIFI_AP_LIST][33];
  char wifi_multi_password[MAX_WIFI_AP_LIST][65];
  char wifi_ap_ssid[33];
  char wifi_ap_password[65];
  bool tcp_socket_mode;
  unsigned int tcp_server_portnum;
  unsigned int tcp_dest_address;
  unsigned int tcp_dest_portnum;
  char default_filename[256];
  int uart_cr_lf_mode;
  unsigned int magic_code;
};
            IPAddress confirm_ip;
            byte dest_ip[4] = {0,};
            dest_ip[0] = serArray[1].toInt();
            dest_ip[1] = serArray[2].toInt();
            dest_ip[2] = serArray[3].toInt();
            dest_ip[3] = serArray[4].toInt();
            IPAddress dest_ip_addr(dest_ip);
            WiFiSerial_config.tcp_dest_address = (uint32_t)dest_ip_addr;
            WiFiSerial_config.tcp_dest_portnum = serArray[5].toInt();
            
*/
  printf("[ Display Device Information :: Serial Monitor ]\n\r\n\r");
  printf("[ WiFi Operation Mode ]\n\r");
  if(WiFiSerial_config.wifi_ap_mode == true) {
    printf("AP mode, ssid:[%s], password:[%s].\n\r", WiFiSerial_config.wifi_ap_ssid, WiFiSerial_config.wifi_ap_password);
  }
  else {
    printf("STATION mode.\n\r");
  }
  printf("\n\r*************************************************************************\n\r");
  printf("[ TCP/IP Remote Connection :: Plain Socket ]\n\r");
  if(WiFiSerial_config.tcp_socket_mode == true) {
    printf("tcp socket mode is : server\n\r");
  }
  else {
    printf("tcp socket mode is : client\n\r");
  }
  printf("server(this) ip address is : 192.168.4.1(fixed)\n\r");
  printf("server(this) socket port number is : %d\n\r", WiFiSerial_config.tcp_server_portnum);
  IPAddress _ip;
  _ip = WiFiSerial_config.tcp_dest_address;
  Serial.print("client(destination) socket IP Address is : ");
  Serial.println(_ip);    
  printf("client(destination) socket port number is : %d\n\r", WiFiSerial_config.tcp_dest_portnum);
  printf("\n\r[ WiFi multi ap list ]\n\r");
  for(int i=0; i<MAX_WIFI_AP_LIST; i++) {
    printf("%d : ssid[%-33s], password[%-16s]\n\r", i, WiFiSerial_config.wifi_multi_ssid[i], WiFiSerial_config.wifi_multi_password[i]);
  }
  printf("Multi ssid update offset = %d\n\r\n\r", WiFiSerial_config.wifi_multi_offset);
  printf("[ Configuration Parameters ]\n\r");
  printf("uart speed = %d\n\r", WiFiSerial_config.uart_speed);
  printf("uart CR/LF mode = %d\n\r", WiFiSerial_config.uart_cr_lf_mode);
  printf("uart rx data log to sd memory(w/ lcd) = %d\n\r", WiFiSerial_config.uart_rx_log);
  printf("socket rx data log to sd memory(w/ lcd) = %d\n\r", WiFiSerial_config.socket_rx_log);
  printf("display to lcd = %d\n\r", WiFiSerial_config.display);
  if(WiFiSerial_config.time_stamp == false) {
    printf("log with time stamp : disable\n\r");
  }
  else {
    printf("log with time stamp : enable\n\r");
  }
  printf("default logging filename = %s\n\r", WiFiSerial_config.default_filename);
  printf("*************************************************************************\n\r\n\r");
}

void do_debug()
{ 
  //시리얼 데이터가 있는지 체크 
  //없으면 그냥 빠져나감.
  if(Serial.available() <= 0)
    return;
    
  //데이터 있으면 1바이트 읽어옴.
  char ch = Serial.read();
      
  Serial.print(ch); //Echo on
  if((ch == '\n') || (ch == '\r'))
  {
    Serial.println();
  }
  if(ch == ',') {
    ch = ' ';
  }
  // 변수에 추가함.
  UartRxString += ch;
  
  // 입력 받은 데이터가 '\n' newline 이면...
  if((ch == '\n') || (ch == '\r'))
  {       
    //여지까지 입력 받은 데이터가 있는지 체크 
    if(UartRxString.length()>0)
    {
      //명령어 시작 위치 찾기
      int startCMD = 0;
      
      //명령어 시작위치를 찾았다면 
      if(startCMD != -1)
      {
        String serArray[10];
        int argcnt=0;
        int idx;

        String tmpString=UartRxString.substring(startCMD);
        //공백 제거 
        tmpString.trim();  
 
        // 데이터 파싱         
        while(tmpString.length() > 0)
        {
          // ' '를 기준으로 짤라서 serArray에 저장.
          idx = tmpString.indexOf(" ");    
          if(idx == -1)
          {
            //','없다면 마지막 데이터 저장후 빠져나감.
            serArray[argcnt] = tmpString;
            serArray[argcnt].trim();       
            argcnt++;                      
            break;
          }
          
          serArray[argcnt] = tmpString.substring(0, idx); 
          tmpString = tmpString.substring(idx+1); 
          tmpString.trim();
          serArray[argcnt].trim();   
                
          argcnt++;              
        }
        //kick deepsleep timer
        if(serArray[0].equalsIgnoreCase("uart"))
        {
          bool is_valid_speed = false;
          int i_speed;

          printf("Set monitoring uart speed.\n\r");
          i_speed = serArray[1].toInt();
          for(int i=0; i<8; i++) {
            if(i_speed == ui_uart_speed[i]) {
              is_valid_speed = true;
              MySerial.begin(i_speed, SERIAL_8N1, RxPin, TxPin);
              WiFiSerial_config.uart_speed = i_speed;
              update_info();
            }
          }
          if(is_valid_speed == false) {
            printf("Unsupported speed, please input valid speed.\n\r");
            printf("usage : uart <9600/19200/38400/57600/115200/230400/460800/921600>\n\r");
          }
        }
        //servo initialize position
        if(serArray[0].equalsIgnoreCase("mode"))
        {
          if(serArray[1].equalsIgnoreCase("ap")) {
            printf("Mode AP configuration!!!.\n\r");       
            wifi_mode_ap = true;
            WiFiSerial_config.wifi_ap_mode = true;
            update_info();
            mode_chage_to_ap();
          }
          else if(serArray[1].equalsIgnoreCase("sta")) {
            printf("Mode STA configuration!!!.\n\r");        
            wifi_mode_ap = false;
            WiFiSerial_config.wifi_ap_mode = false;
            update_info();            
            mode_chage_to_sta();
          }                 
          else {
            printf("Command error. Please select 'ap' or 'sta' mode.\n\r");
            printf("usage : mode <ap/sta>\n\r");
          }
        }
        if(serArray[0].equalsIgnoreCase("multista")) //Add ap
        {
          if(serArray[1].length() > 0 && serArray[2].length() > 0) { //ssid
            int write_offset;
            write_offset = WiFiSerial_config.wifi_multi_offset;
            printf("Set wifi ap mode information(overwrite).!!!.\n\r");
            //Init ssid and password
            for(int i=0; i<33; i++){
              WiFiSerial_config.wifi_multi_ssid[write_offset][i] = NULL;
            }
            for(int i=0; i<65; i++){
              WiFiSerial_config.wifi_multi_password[write_offset][i] = NULL;
            }
            //copy ssid, password
            strcpy(WiFiSerial_config.wifi_multi_ssid[write_offset], serArray[1].c_str());
            strcpy(WiFiSerial_config.wifi_multi_password[write_offset], serArray[2].c_str());
            write_offset++;
            if( write_offset >= MAX_WIFI_AP_LIST) {
              write_offset = 0;
            }
            WiFiSerial_config.wifi_multi_offset = write_offset;
            update_info();
          }
          else {
            printf("Command error. Please input 'ssid' and 'password'.\n\r");
            printf("usage : multista <ssid> <password>\n\r");
          }                
        }
        if(serArray[0].equalsIgnoreCase("ap")) //Add ap
        {
          if(serArray[1].length() > 0 && serArray[2].length() > 0) { //ssid
            int write_offset;
            write_offset = WiFiSerial_config.wifi_multi_offset;
            printf("Set wifi sta mode information.!!!.\n\r");
            //Init ssid and password
            for(int i=0; i<33; i++){
              WiFiSerial_config.wifi_ap_ssid[i] = NULL;
            }
            for(int i=0; i<65; i++){
              WiFiSerial_config.wifi_ap_password[i] = NULL;
            }
            //copy ssid, password
            strcpy(WiFiSerial_config.wifi_ap_ssid, serArray[1].c_str());
            strcpy(WiFiSerial_config.wifi_ap_password, serArray[2].c_str());
            update_info();
          }
          else {
            printf("Command error. Please input 'ssid' and 'password' and 'port number'.\n\r");
            printf("usage : ap <ssid> <password> <port number>\n\r");
          }                
        }        
        if(serArray[0].equalsIgnoreCase("tcp"))
        {
          if(serArray[1].equalsIgnoreCase("server")) {
            WiFiSerial_config.tcp_socket_mode = true; //server mode
            update_info();
          }
          else if(serArray[1].equalsIgnoreCase("client")) {
            WiFiSerial_config.tcp_socket_mode  = false; //client mode
            update_info();
          }
          else {
            printf("usage : tcp <server/client>\n\r");
          }
        }
        if(serArray[0].equalsIgnoreCase("server"))
        {
          if(argcnt == 2) {
            WiFiSerial_config.tcp_server_portnum = serArray[1].toInt();
            SERVER_PORT = WiFiSerial_config.tcp_server_portnum;
            update_info();
          }
          else {
            printf("usage : server <listening port number>\n\r");
          }
        }
        if(serArray[0].equalsIgnoreCase("client"))
        {
          if(argcnt == 6) {
            /*
              unsigned int tcp_dest_address;
              unsigned int tcp_dest_portnum;
            */
            //ip address
            //IPAddress IP = IPAddress (10, 10, 2, 6)
            IPAddress confirm_ip;
            byte dest_ip[4] = {0,};
            dest_ip[0] = serArray[1].toInt();
            dest_ip[1] = serArray[2].toInt();
            dest_ip[2] = serArray[3].toInt();
            dest_ip[3] = serArray[4].toInt();
            IPAddress dest_ip_addr(dest_ip);
            WiFiSerial_config.tcp_dest_address = (uint32_t)dest_ip_addr;
            WiFiSerial_config.tcp_dest_portnum = serArray[5].toInt();
            confirm_ip = WiFiSerial_config.tcp_dest_address;
            Serial.print("tcp socket destination server ip address=");
            Serial.print(confirm_ip);
            Serial.print(", port=");
            Serial.println(WiFiSerial_config.tcp_dest_portnum);
            update_info();
          }
          else {
            printf("usage : client <ip address octet1> <ip address octet2> <ip address octet3> <ip address octet4> <port number>\n\r");
          }
        }
        //  printf("uart CR/LF mode = %d\n\r", WiFiSerial_config.uart_cr_lf_mode);
        if(serArray[0].equalsIgnoreCase("crlf"))
        {
          if(argcnt == 2) {
            if((serArray[1].toInt() >= 0) && (serArray[1].toInt() <= 2)) {
              WiFiSerial_config.uart_cr_lf_mode = serArray[1].toInt();
              update_info();
            }
            else {
              printf("Command error. Please input crlf (0:no change, 1:CR to LF, 2:CR to CRLF) example.\n\r");
              printf("usage : crlf <0/1/2>\n\r");
            }
          }
          else {
            printf("Command error. Please input crlf (0:no change, 1:CR to LF, 2:CR to CRLF) example.\n\r");
            printf("usage : crlf <0/1/2>\n\r");
          }
          
        }
        //WiFiSerial_config.time_stamp
        if(serArray[0].equalsIgnoreCase("timestamp"))
        {
          if(argcnt == 2) {
            if(serArray[1].equalsIgnoreCase("enable")) {
              WiFiSerial_config.time_stamp = true;
              update_info();
            }
            else if(serArray[1].equalsIgnoreCase("disable")) {
              WiFiSerial_config.time_stamp = false;
              update_info();
            }
          }
          else {
            printf("usage : timestamp <enable/disable>\n\r");
          }
        }      
        /*
          printf("log to uart rx data in sd memory(w/ lcd) : uartlog <0/1>\n\r");//, WiFiSerial_config.uart_rx_log);
          printf("log to socket rx data in sd memory(w/ lcd) : socketlog <0/1>\n\r");//, WiFiSerial_config.socket_rx_log);  
        */
        if(serArray[0].equalsIgnoreCase("uartlog"))
        {
          if(argcnt == 2) {
            if(serArray[1].equalsIgnoreCase("1")) {
              WiFiSerial_config.uart_rx_log = true;
              update_info();
            }
            else if(serArray[1].equalsIgnoreCase("0")) {
              WiFiSerial_config.uart_rx_log = false;
              update_info();
            }
          }
          else {
            printf("usage : uartlog <0/1>\n\r");
          }
        }
        if(serArray[0].equalsIgnoreCase("socketlog"))
        {
          if(argcnt == 2) {
            if(serArray[1].equalsIgnoreCase("1")) {
              WiFiSerial_config.socket_rx_log = true;
              update_info();
            }
            else if(serArray[1].equalsIgnoreCase("0")) {
              WiFiSerial_config.socket_rx_log = false;
              update_info();
            }
          }
          else {
            printf("usage : uartlog <0/1>\n\r");
          }
        }        
        if(serArray[0].equalsIgnoreCase("display"))
        {
          if(argcnt == 2) {
            if(serArray[1].equalsIgnoreCase("1")) {
              WiFiSerial_config.display = true;
              update_info();
            }
            else if(serArray[1].equalsIgnoreCase("0")) {
              WiFiSerial_config.display = false;
              update_info();
            }
          }
          else {
            printf("usage : display <0/1>\n\r");
          }
        }            
        if(serArray[0].equalsIgnoreCase("help") || serArray[0].equalsIgnoreCase("?"))
        {
          printf("[debug command for monitoring and logging with wifi connectivity device]\n\r");
          printf("monitoring uart speed : uart <9600/19200/38400/57600/115200/230400/460800/921600>\n\r");
          printf("monitoring uart CR/LF handling(0:no change, 1:CR to LF, 2:CR to CRLF) : crlf <0/1/2>\n\r");
          printf("log to uart rx data in sd memory(w/ lcd) : uartlog <0/1>\n\r");//, WiFiSerial_config.uart_rx_log);
          printf("log to socket rx data in sd memory(w/ lcd) : socketlog <0/1>\n\r");//, WiFiSerial_config.socket_rx_log);        
          printf("display to lcd on/off : display <0/1>\n\r");
          printf("select operation mode : mode <ap/sta>\n\r");
          printf("WiFi ap mode information : ap <ssid> <password>\n\r");
          printf("WiFi sta mode information(max %d wifiap supported)  : multista <ssid> <password>\n\r", MAX_WIFI_AP_LIST);
          printf("TCP socket operation mode : tcp <server/client>\n\r");
          printf("TCP socket server mode : server <listening port number>\n\r");
          printf("TCP socket client mode : client <ip address octet1> <ip address octet2> <ip address octet3> <ip address octet4> <port number>\n\r");
          printf("device ip address and tcp socket server port number display : ipaddr\n\r");
          printf("factory default NV : factory\n\r");
          printf("device configuration display : info\n\r");
          printf("time data display or seeting(rtc, NTP or manual support) : rtc <year:2024> <month:9> <day:30> <hour:14> <minute:58> <second:12>\n\r");
          printf("log with time stamp : timestamp <enable/disable>\n\r");
          printf("default logging filename(*.txt) : logfile <filename>\n\r");
          printf("lcd display output : lcd <on/off>\n\r");
          printf("restart device : reset\n\r");
        }
        if(serArray[0].equalsIgnoreCase("factory"))
        {
          clear_nv();
          update_info();
        }
        if(serArray[0].equalsIgnoreCase("ipaddr"))
        {
          if(wifi_mode_ap == true) {
            MessageLcd =  MyWifiSsid + " : " + WiFi.softAPIP().toString() + ":" + String(SERVER_PORT);
          }
          else {
            MessageLcd =  WiFi.SSID() + " : " + WiFi.localIP().toString() + ":" + String(SERVER_PORT);
          }
        }        
        if(serArray[0].equalsIgnoreCase("rtc"))
        {
          struct tm timeinfo;
          struct tm timeset;
          time_t now = time(nullptr);
          if(argcnt == 1) {
            localtime_r (&now, &timeinfo);
            Serial.print ("Current time : ");
            Serial.print (asctime (&timeinfo));
          }
          else if(argcnt == 7){
            timeset.tm_year = (serArray[1].toInt()) - 1900;
            timeset.tm_mon = serArray[2].toInt() - 1;
            timeset.tm_mday = serArray[3].toInt();
            timeset.tm_hour = serArray[4].toInt();
            timeset.tm_min = serArray[5].toInt();
            timeset.tm_sec = serArray[6].toInt();
            time_t t = mktime(&timeset);
            struct timeval now = { .tv_sec = t };
            settimeofday(&now, NULL);            
          }
          else {
            printf("Command error. Please input 'rtc' or 'rtc 2024 9 4 9 12 13' example.\n\r");
            printf("usage : rtc / rtc <year:2024> <month:9> <day:30> <hour:14> <minute:58> <second:12>\n\r");
          }
        }
        if(serArray[0].equalsIgnoreCase("info"))
        {
          view_boot_message();
          printf("\nTotal boot count = %d\n\r", WiFiSerial_config.boot_count);
          display_serialwifi_info();
        }
        if(serArray[0].equalsIgnoreCase("logfile"))
        {
          //view_boot_message();
          if(argcnt == 2) {
            if(serArray[1].length() > 0) {
              String m_filename;
              m_filename = "/" + serArray[1]; //Add '/' for root directory.
              strcpy(WiFiSerial_config.default_filename, m_filename.c_str());
              printf("default logging filename = %s\n\r", WiFiSerial_config.default_filename);
              init_sd_card();
              update_info();
            }
          }
          else {
            printf("Command error. Please input 'logfile' 'filename'.\n\r");
            printf("usage : logfile <filename>\n\r");
          }
        }        
        //reset
        if(serArray[0].equalsIgnoreCase("reset"))
        {
          myshutdown();
        }     
      }
    }
    UartRxString = "";
  }   
}

void init_sd_card()
{
  Serial.print("Initializing SD card...");
 
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  get_FileName(FileName);
  Serial.print("Creating: ");
  Serial.print(FileName);
  Serial.print("...");
  f_name = "/" + String(FileName);
  File file = SD.open(f_name);//"/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    if(wifi_mode_ap == false) {
      writeFile(SD, f_name.c_str() , "Date, Time, Serial Data Logging \r\n");
    }
    else
    {
      writeFile(SD, f_name.c_str() , "Serial Data Logging \r\n");
    }
  }
  else {
    Serial.println("File already exists");  
  }  
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  /*
  File file = SD.open(f_name.c_str());//"/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    if(wifi_mode_ap == false) {
      writeFile(SD, f_name.c_str() , "Date, Time, Serial Data Logging \r\n");
    }
    else
    {
      writeFile(SD, f_name.c_str() , "Serial Data Logging \r\n");
    }
  }
  else {
    Serial.println("File already exists");  
  }
  */
  file.close();  
}

void myshutdown()
{
  EEPROM.end();
  Serial.print("\n\rReboot Device... Copyright(c) 2023 Woo Hyuk Joon*******\n\n\n\r\r\r");
  ESP.restart(); //ESP32
}

void send_google_mail()
{

}

//For send email :: https://msr-r.net/m5stack-send-email/
#include <EMailSender.h>

char myMailAdr[100]  = "uhj0305@gmail.com";
char myMailPass[100] = "xxmi ooir hvey seph";//"google 설정->보안->2단계인증->앱 비밀번호에서 등록한 장치의 키";
char toMailAdr[100]  = "uhj0305@gmail.com";
String strDeviceInfo;

EMailSender emailSend(myMailAdr, myMailPass);
String MkMsg;
String AddMsg = "";
boolean send_mail(String msg) {
  EMailSender::EMailMessage message;

  uint64_t chipid;
  String strDeviceInfoUcase;

  chipid=ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes)
  //Serial.printf("\n\rESP32 Chip ID = %04X", (uint16_t)(chipid>>32)); // print High 2 bytes
  //Serial.printf("%016X\n\r", chipid); // print Low 4bytes        
  strDeviceInfoUcase = String(chipid, HEX);
  strDeviceInfoUcase.toUpperCase();
  strDeviceInfo = strDeviceInfoUcase;
  strDeviceInfoUcase = "ESP32 Chip ID :: " + strDeviceInfoUcase + " Config Data Report.";
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  message.subject = strDeviceInfoUcase;
  get_email_msg();
  message.message = AddMsg;
  EMailSender::Response resp = emailSend.send(toMailAdr, message);

  return true;
}

char tmp_string[256];
void get_email_msg()
{
  IPAddress client_destination_ip;
  //echo "<pre>\t\tTABS!\t\t</pre>";'
  /*
  If you are sending HTML email then use <BR> (or <BR />, or </BR>) as stated.
  If you are sending a plain text email then use %0D%0A
  \r = %0D (Ctrl+M = carriage return)
  \n = %0A (Ctrl+A = line feed)
struct config_data{
  unsigned int boot_count;
  bool wifi_ap_mode;
  unsigned int uart_speed;
  unsigned int wifi_multi_offset;
  char wifi_multi_ssid[MAX_WIFI_AP_LIST][33];
  char wifi_multi_password[MAX_WIFI_AP_LIST][65];
  char wifi_ap_ssid[33];
  char wifi_ap_password[65];
  bool tcp_socket_mode;
  unsigned int tcp_server_portnum;
  unsigned int tcp_dest_address;
  unsigned int tcp_dest_portnum;
  bool time_stamp;
  char default_filename[256];
  int uart_cr_lf_mode;
  unsigned int magic_code;
};  
  */
  sprintf(tmp_string, "[ Device Information ID:0x%s ]<BR>", strDeviceInfo.c_str());
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  MkMsg = "WiFi " + WiFi.SSID() + " ip address " + WiFi.localIP().toString() + ":" + String(SERVER_PORT) + " Connected.<BR>";
  AddMsg += MkMsg;
  sprintf(tmp_string, "Boot count = %d<BR>", WiFiSerial_config.boot_count);
  MkMsg = tmp_string;
  AddMsg += MkMsg;

  sprintf(tmp_string, "[ Device Wifi Operation Mode Information ]<BR>");
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  if(WiFiSerial_config.wifi_ap_mode == true) {
    sprintf(tmp_string, "Wifi operation mode is : ap<BR>");
  }
  else {
    sprintf(tmp_string, "Wifi operation mode is : station<BR>");
  }
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "Wifi info(ap mode) is : [%s] [%s]<BR>", WiFiSerial_config.wifi_ap_ssid, WiFiSerial_config.wifi_ap_password);
  MkMsg = tmp_string;
  AddMsg += MkMsg;  
  sprintf(tmp_string, "<BR>[ WiFi Multi AP(STATION scan and connect) list ]<BR>");
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  for(int i=0; i<MAX_WIFI_AP_LIST; i++) {
    sprintf(tmp_string, "%d : [%s] [%s]<BR>", i, WiFiSerial_config.wifi_multi_ssid[i], WiFiSerial_config.wifi_multi_password[i]);
    MkMsg = tmp_string;
    AddMsg += MkMsg;
  } 
  sprintf(tmp_string, "Multi ssid overwrite offset = %d<BR><BR>", WiFiSerial_config.wifi_multi_offset);
  MkMsg = tmp_string;
  AddMsg += MkMsg;

  sprintf(tmp_string, "[ TCP/IP Remote Connection Information ]<BR>");
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  if(WiFiSerial_config.tcp_socket_mode == true) {
    sprintf(tmp_string, "tcp socket mode is : server<BR>");
  }
  else {
    sprintf(tmp_string, "tcp socket mode is : client<BR>");
  }
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "server(this) ip address is : 192.168.4.1(fixed)<BR>");
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "server(this) socket port number is : %d<BR>", WiFiSerial_config.tcp_server_portnum);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  IPAddress _ip;
  _ip = WiFiSerial_config.tcp_dest_address;
  sprintf(tmp_string, "client(destination) socket ip address is : %d.%d.%d.%d<BR>", _ip[0], _ip[1], _ip[2], _ip[3]);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "client(destination) socket port number is : %d<BR>", WiFiSerial_config.tcp_dest_portnum);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "<BR>[ Configuration Parameters ]<BR>");
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "Uart speed = %d<BR>", WiFiSerial_config.uart_speed);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "Uart cr/lf mode = %d<BR>", WiFiSerial_config.uart_cr_lf_mode);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  sprintf(tmp_string, "Log with timestamp = %d<BR>", WiFiSerial_config.time_stamp);
  MkMsg = tmp_string;
  AddMsg += MkMsg;  
  //printf("lcd display output : lcd <on/off>\n\r");
  
  sprintf(tmp_string, "default logging filename = %s<BR>", WiFiSerial_config.default_filename);
  MkMsg = tmp_string;
  AddMsg += MkMsg;
  //Serial.print(AddMsg);
}

void draw_status(bool flag)
{
  if(flag == true) {
      M5.Lcd.fillCircle(5, 8, 5, 0x07E0);
  }
  else {
    M5.Lcd.fillCircle(5, 8, 5, 0xF800);
  }
}
////////////////////////////////////////////////////////////////////////////////////
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        Serial.print("  LEVEL S : ");
        Serial.println(levels);
        listDir(fs, file.path(), levels - 1);
        Serial.print("  LEVEL E : ");
        Serial.println(levels);        
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}



void get_FileName(char *FileName)
{
      
      FileName[0] = 'L';
      FileName[1] = 'o';
      FileName[2] = 'g';
      FileName[3] = '_';      
      FileName[4] = '0';
      FileName[5] = '0';
      FileName[6] = '0';
      FileName[7] = '0';
      FileName[8] = '.';
      FileName[9] = 't';
      FileName[10] = 'x';
      FileName[11] = 't';
      
      
      for(int i=0;i<1000;i++)
      {   
          FileName[4] = (i/1000)%10 + '0';
          FileName[5] = (i/100)%10 + '0';  
          FileName[6] = (i/10)%10 + '0'; 
          FileName[7] = i%10 + '0';
          
          Serial.print("Searching for ");
          Serial.println(FileName);
          f_name = "/" + String(FileName);
          if (!SD.exists(f_name)) 
          {
              Serial.print("Chosen Name: ");
              Serial.println(FileName); 
              return;
          }
          else
          {
              Serial.print(FileName);
              Serial.println(" already exists!");
          }
      }
}

/*
#include <ESPAsyncWebSrv.h>
const char* PARAM_MESSAGE = "file";
// Create AsyncWebServer object on port 80
AsyncWebServer web_server(80);

void initSDCard(){
  if(!SD.begin(chipSelect)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void webserver() {
  initSDCard();

  web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    String ftype;
    if (request->hasParam(PARAM_MESSAGE)) {
        message = request->getParam(PARAM_MESSAGE)->value();
        message.replace('~', '/');
    }
    if (request->hasParam("ftype")) {
        ftype = request->getParam("ftype")->value();
        ftype.replace('~', '/');
    }
    request->send(SD, "/"+message, ftype);
    Serial.printf("User requested file '%s' using file type '%s'\n", message, ftype);
  });

  web_server.serveStatic("/", SD, "/");

  web_server.begin();
}
*/

//#include <M5Stack.h>
//#include <WiFi.h>
//#include <WiFiClient.h>

#if 0
String getContentType(String filename) {
  filename.toUpperCase();
  if (web_server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".HTM")) return "text/html";
  else if (filename.endsWith(".HTML")) return "text/html";
  else if (filename.endsWith(".CSS")) return "text/css";
  else if (filename.endsWith(".JS")) return "application/javascript";
  else if (filename.endsWith(".PNG")) return "image/png";
  else if (filename.endsWith(".GIF")) return "image/gif";
  else if (filename.endsWith(".JPG")) return "image/jpeg";
  else if (filename.endsWith(".ICO")) return "image/x-icon";
  else if (filename.endsWith(".XML")) return "text/xml";
  else if (filename.endsWith(".PDF")) return "application/x-pdf";
  else if (filename.endsWith(".ZIP")) return "application/x-zip";
  else if (filename.endsWith(".GZ")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SD.exists(pathWithGz) || SD.exists(path)) {
    if (SD.exists(pathWithGz))
      path += ".gz";
    File file = SD.open(path, "r");
    size_t sent = web_server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleRoot() {
  //GO.lcd.clear();
  //GO.lcd.setCursor(0, 0);

  String directory = urldecode(web_server.uri());
  uploadPath = directory;
  File dir = SD.open(directory);

  //GO.lcd.print("Index of ");GO.lcd.println(directory);

  String entryName = "";
  String tree = "";
  bool emptyFolder = true;
  while (true) {
    File entry =  dir.openNextFile();
    entryName = entry.name();
    entryName.replace(directory + "/", "");

    if (! entry) {
      // no more files
      break;
    }

    if (entry.isDirectory()) {
      tree += F("<tr>");
      tree += F("<td data-value=\"");
      tree += entryName;
      tree += F("/\"><a class=\"icon dir\" href=\"");
      tree += entry.name();
      tree += F("\">");
      tree += entryName;
      tree += F("/</a></td>");
      tree += F("<td class=\"detailsColumn\" data-value=\"0\">-</td>");
      tree += F("<td class=\"detailsColumn\" data-value=\"0\">");
      tree += F("<button class='buttons' onclick=\"location.href='/deleteConfirm?folder=");
      tree += entry.name();
      tree += F("';\">Delete</button></td>");
      tree += F("</tr>");
      //GO.lcd.print(entryName); GO.lcd.println(" - <DIR>");
      emptyFolder = false;
    } else {
      tree += F("<tr>");
      tree += F("<td data-value=\"");
      tree += entry.name();
      tree += F("\"><a class=\"icon file\" draggable=\"true\" href=\"");
      tree += entry.name();
      tree +=  F("\">");
      tree += entryName;
      tree += F("</a></td>");
      tree += F("<td class=\"detailsColumn\" data-value=\")");
      tree += file_size(entry.size());
      tree += F("\">");
      tree += file_size(entry.size());
      tree += F("</td>");
      tree += F("<td class=\"detailsColumn\" data-value=\"0\">");
      tree += F("<button class='buttons' onclick=\"location.href='/deleteConfirm?file=");
      tree += entry.name();
      tree += F("';\">Delete</button></td>");
      tree += F("</tr>");
      //GO.lcd.print(entryName); GO.lcd.print(" - "); GO.lcd.println(file_size(entry.size()));
      emptyFolder = false;
    }
    entry.close();
  }

  int i, count;
  for (i = 0, count = 0; directory[i]; i++)
    count += (directory[i] == '/');
  count++;

  int parserCnt = 0;
  int rFromIndex = 0, rToIndex = -1;
  String lastElement = "";
  String tempElement = "";
  String path = directory;
  path.remove(0, 1);
  path += "/";
  while (count >= parserCnt) {
    rFromIndex = rToIndex + 1;
    rToIndex = path.indexOf('/', rFromIndex);
    if (count == parserCnt) {
      if (rToIndex == 0 || rToIndex == -1) break;
      tempElement = lastElement;
      lastElement = path.substring(rFromIndex, rToIndex);
    } else parserCnt++;
  }
  /*
    Serial.print("directory:");
    Serial.println(directory);
    Serial.print("path:");
    Serial.println(path);
    Serial.print("lastElement:");
    Serial.println(lastElement);
  */
  String webpage = "";
  webpage += F(header);
  webpage += F("<h1 id=\"header\">Index of ");
  webpage += directory;
  webpage += F("</h1>");

  if (directory != "/") {
    webpage += F("<div id=\"parentDirLinkBox\" style=\"display:block;\">");
    webpage += F("<a id=\"parentDirLink\" class=\"icon up\" href=\"");
    directory.replace(lastElement, "");
    if (directory.length() > 1)
      directory = directory.substring(0, directory.length() - 1);
    webpage += directory;
    webpage += F("\">");
    webpage += F("<span id=\"parentDirText\">[Parent directory]</span>");
    webpage += F("</a>");
    webpage += F("</div>");
  }

  webpage += F(script);

  webpage += F("<table>");
  webpage += F("<thead>");
  webpage += F("<tr class=\"header\" id=\"theader\">");
  webpage += F("<th onclick=\"sortTable(0);\">Name</th>");
  webpage += F("<th class=\"detailsColumn\" onclick=\"sortTable(1);\">Size</th>");
  webpage += F("<th></th>");
  webpage += F("</tr>");
  webpage += F("</thead>");
  webpage += F("<tbody id=\"tbody\">");
  webpage += tree;
  webpage += F("</tbody>");
  webpage += F("</table>");
  webpage += F("<hr>");

  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' type='file' name='fupload' id = 'fupload' value=''>");
  webpage += F("<button class='buttons' type='submit'>Upload</button></form><br>");

  webpage += F("<FORM action='/mkdir' method='post' enctype='multipart/form-data'>");
  webpage += F("<input type='hidden' id='path' name='path' value='");
  webpage += uploadPath;
  webpage += F("'>");
  webpage += F("<input class='buttons' name='dirName' id ='dirName' value='NewFolder'>");
  webpage += F("<button class='buttons' type='submit'>MkDir</button></form>");
  webpage += F(footer);

  if (tree == "") {
    String dlPath = urldecode(web_server.uri());
    if (SD.exists(dlPath)) {
      File entry = SD.open(dlPath);
      if (!entry.isDirectory()) {
        Serial.println(dlPath);
        handleFileRead(dlPath);
      }
    }
    else {
      handleNotFound();
    }
  }

  web_server.send(200, "text/html", webpage);
}

void handleNotFound() {
  String webpage = "";
  webpage += F(header);
  webpage += F("<hr>File Not Found<br>");
  webpage += F("<br>URI:");
  webpage += web_server.uri();
  webpage += F("<br>Method: ");
  webpage += (web_server.method() == HTTP_GET) ? "GET" : "POST";
  webpage += F("<br>Arguments: ");
  webpage += web_server.args();
  webpage += F("<br>");
  for (uint8_t i = 0; i < web_server.args(); i++) {
    webpage += web_server.argName(i) + ": " + web_server.arg(i) + "<br>";
  }
  webpage += F("<br><button class='buttons' onclick=\"location.href='/';\">OK</button>");
  webpage += F(footer);
  web_server.send(404, "text/html", webpage);
}

void doMkdir() {
  //GO.lcd.clear();
  //GO.lcd.setCursor(0, 0);

  String webpage = "";
  webpage += F(header);
  String path = "";
  String dirName = "";

  for (uint8_t i = 0; i < web_server.args(); i++) {
    if (web_server.argName(i) == "dirName") {
      Serial.printf("Dir Name: %s\n", web_server.arg(i));
      dirName =  web_server.arg(i);
    }
    if (web_server.argName(i) == "path") {
      Serial.printf("Path: %s\n", web_server.arg(i));
      path = web_server.arg(i);
    }

  }

  if (dirName != "" && path != "") {
    webpage += F("<hr>Creating Dir: <br>");

    if (path == "/")
      path = path + dirName;
    else
      path = path + "/" + dirName;

    webpage += path;

    Serial.printf("Creating Dir: %s\n", path);
    //GO.lcd.print("Creating Dir:"); GO.lcd.println(path);
    if (SD.mkdir(path)) {
      webpage += F("<br>Dir created<br>");
      //GO.lcd.println("Dir created");
    } else {
      Serial.println("mkdir failed");
      webpage += F("<br>mkdir failed<br>");
      //GO.lcd.println("mkdir failed");
    }
  } else {
    webpage += F("<br>Path or Name empty!");
    //GO.lcd.println("Path or Name empty!");
  }

  webpage += F("<br><button class='buttons' onclick=\"location.href='/';\">OK</button>");
  webpage += F(footer);
  web_server.send(200, "text/html", webpage);
}

void doDelete() {
  //GO.lcd.clear();
  //GO.lcd.setCursor(0, 0);

  String webpage = "";
  String d_name;
  webpage += F(header);
  for (uint8_t i = 0; i < web_server.args(); i++) {
    if (web_server.argName(i) == "file") {
      Serial.printf("Deleting file: %s\n", web_server.arg(i));
      //GO.lcd.print("Deleting file:"); GO.lcd.println(web_server.arg(i));
      webpage += F("<hr>Deleting file: <br>");
      webpage += web_server.arg(i);
      d_name = "/" + web_server.arg(i);
      if (SD.remove(d_name.c_str())){//web_server.arg(i))) {
        webpage += F("<br>File deleted<br>");
        //GO.lcd.println("File deleted");
      } else {
        webpage += F("<br>Delete failed<br>");
        //GO.lcd.println("Delete failed");
      }
    }
    if (web_server.argName(i) == "folder") {
      Serial.printf("Removing Dir: %s\n", web_server.arg(i));
      //GO.lcd.print("Removing Dir:"); GO.lcd.println(web_server.arg(i));
      webpage += F("<hr>Removing Dir: <br>");
      webpage += web_server.arg(i);
      if (SD.rmdir(web_server.arg(i))) {
        webpage += F("<br>Dir removed<br>");
        //GO.lcd.println("Dir removed");
      } else {
        webpage += F("<br>rmdir failed<br>");
        //GO.lcd.println("rmdir failed");
      }
    }
  }
  webpage += F("<br><button class='buttons' onclick=\"location.href='/';\">OK</button>");
  webpage += F(footer);
  web_server.send(200, "text/html", webpage);
}

void deleteConfirm() {
  String webpage = "";
  webpage += F(header);
  for (uint8_t i = 0; i < web_server.args(); i++) {
    if (web_server.argName(i) == "file") {
      webpage += F("<hr>Do you want to delete the file:<br>");
      webpage += web_server.arg(i);
      webpage += F("<br><br><button class='buttons' onclick=\"location.href='/doDelete?file=");
      webpage += web_server.arg(i);
      webpage += F("';\">Yes</button>");

    }
    if (web_server.argName(i) == "folder") {
      webpage += F("<hr>Do you want to delete the Directory:<br>");
      webpage += web_server.arg(i);
      webpage += F("<br><br><button class='buttons' onclick=\"location.href='/doDelete?folder=");
      webpage += web_server.arg(i);
      webpage += F("';\">Yes</button>");
    }
  }

  webpage += F("<button class='buttons' onclick='window.history.back();'>No</button>");
  webpage += F(footer);
  web_server.send(200, "text/html", webpage);
}

File UploadFile;
void handleFileUpload() {
  // upload a new file to the Filing system
  HTTPUpload& uploadfile = web_server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = uploadPath + "/" + filename;
    //GO.lcd.clear();
    //GO.lcd.setCursor(0, 0);
    Serial.print("Upload File Name: "); Serial.println(filename);
    //GO.lcd.print("Upload File Name: "); GO.lcd.println(filename);
    SD.remove(filename);                         // Remove a previous version, otherwise data is appended the file again
    UploadFile = SD.open(filename, FILE_WRITE); // Open the file for writing in SPIFFS (create it, if doesn't exist)
    filename = String();
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) {
      UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
      //GO.lcd.setCursor(0, 30);
      //GO.lcd.fillRect(0, 29, TFT_HEIGHT, 20, BLACK);
      //GO.lcd.print("Loading: "); GO.lcd.println(file_size(uploadfile.totalSize));
    }
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      delay(1000);
      //GO.lcd.clear();
      //GO.lcd.setCursor(0, 0);
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      //GO.lcd.println("File was successfully uploaded.");
      //GO.lcd.print("Uploaded File Name: "); GO.lcd.println(uploadfile.filename);
      //GO.lcd.print("File Size: "); GO.lcd.println(file_size(uploadfile.totalSize));

      String webpage = "";
      webpage += F(header);
      webpage += F("<hr>File was successfully uploaded<br>");
      webpage += F("Uploaded File Name: ");
      webpage += uploadfile.filename + "<br>";
      webpage += F("File Size: ");
      webpage += file_size(uploadfile.totalSize) + "<br>";
      webpage += "<button class='buttons' onclick='window.history.back();'>OK</button>";
      webpage += F(footer);
      web_server.send(200, "text/html", webpage);
    }
    else
    {
      delay(1000);
      //GO.lcd.clear();
      //GO.lcd.setCursor(0, 0);
      //GO.lcd.println("Could Not Create Uploaded File (write-protected?)");
      String webpage = "";
      webpage += F(header);
      webpage += F("<hr>Could Not Create Uploaded File (write-protected?)<br>");
      webpage += "<button class='buttons' onclick='window.history.back();'>OK</button>";
      webpage += F(footer);
      web_server.send(200, "text/html", webpage);
    }
  }
}

String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))      fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                              fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}

String urldecode(String str)
{

  String encodedString = "";
  char c;
  char code0;
  char code1;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == '+') {
      encodedString += ' ';
    } else if (c == '%') {
      i++;
      code0 = str.charAt(i);
      i++;
      code1 = str.charAt(i);
      c = (h2int(code0) << 4) | h2int(code1);
      encodedString += c;
    } else {

      encodedString += c;
    }

    yield();
  }

  return encodedString;
}

String urlencode(String str)
{
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      //encodedString+=code2;
    }
    yield();
  }
  return encodedString;

}

unsigned char h2int(char c)
{
  if (c >= '0' && c <= '9') {
    return ((unsigned char)c - '0');
  }
  if (c >= 'a' && c <= 'f') {
    return ((unsigned char)c - 'a' + 10);
  }
  if (c >= 'A' && c <= 'F') {
    return ((unsigned char)c - 'A' + 10);
  }
  return (0);
}

String split(String s, char parser, int index) {
  String rs = "";
  int parserIndex = index;
  int parserCnt = 0;
  int rFromIndex = 0, rToIndex = -1;
  while (index >= parserCnt) {
    rFromIndex = rToIndex + 1;
    rToIndex = s.indexOf(parser, rFromIndex);
    if (index == parserCnt) {
      if (s.substring(rFromIndex, rToIndex) == "")
        break;
    } else parserCnt++;
    rs = s.substring(rFromIndex, rToIndex);
  }
  return rs;
}

void webserver_setup(void) {
  //GO.begin();
  //GO.Speaker.setVolume(0);
  //pinMode(25, OUTPUT);
  //digitalWrite(25, LOW);
  /*
    M5.begin();
    M5.Power.begin();
    // M5.Lcd.setRotation(5); // Must be setRotation(0) for this sketch to work
    // correctly
    M5.Lcd.fillScreen(TFT_BLACK);
*/
    // Setup baud rate and draw top banner
    //Serial.begin(115200);
  //GO.battery.setProtection(true);
  //delay(100);

  //GO.lcd.clear();
  //GO.lcd.setCursor(0, 0);
  //GO.lcd.setTextWrap(false);

  Serial.println("");

  Serial.print("Initializing SD card...");
  //GO.lcd.println("Initializing SD card...");

  if (!SD.begin()) {
    //GO.lcd.println("Card Mount Failed");
    Serial.println("Card Mount Failed");
    //GO.update();
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    //GO.lcd.println("No SD card attached");
    Serial.println("No SD card attached");
    //GO.update();
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    //GO.lcd.println("MMC");
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    //GO.lcd.println("SDSC");
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    //GO.lcd.println("SDHC");
    Serial.println("SDHC");
  } else {
    //GO.lcd.println("UNKNOWN");
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  //GO.lcd.printf("SD Card Size: %lluMB\n", cardSize);

  //GO.lcd.println(" ");
  Serial.println();

  //GO.lcd.println("-Wifi Mode-");
  //GO.lcd.println("-> Press A for Access Point mode");
  //GO.lcd.println("-> Press B to connect to a Wifi");
  //GO.lcd.println(" ");
  Serial.println();
  while (0){//true) {
    if (1){//GO.BtnA.isPressed()) {
      // You can remove the password parameter if you want the AP to be open.
      WiFi.softAP(ssid, password);
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("AP IP address: "); Serial.println(myIP);
      //GO.lcd.println("AP started");
      //GO.lcd.print("AP SSID: "); GO.lcd.println(ssid);
      //GO.lcd.print("AP Password: "); GO.lcd.println(password);
      //GO.lcd.print("AP IP address: "); GO.lcd.println(myIP);
      break;
    }
    if (0){//GO.BtnB.isPressed()) {
      String WifiSSID = "";
      String WifiPSK = "";

      String path = "/WIFI.TXT";
      Serial.print("Reading file: ");
      Serial.println(path);
      //GO.lcd.print("Reading file: ");
      ///GO.lcd.println(path);
      File wifiFile = SD.open(path);
      if (!wifiFile) {
        Serial.println("Failed to open file for reading");
        //GO.lcd.println("Failed to open file for reading");
        return;
      }
      Serial.println("Read from file: ");
      //GO.lcd.println("Read from file: ");
      while (wifiFile.available()) {
        WifiSSID = wifiFile.readStringUntil('\n');
        WifiSSID.replace("\r", "");
        WifiPSK = wifiFile.readStringUntil('\n');
        WifiPSK.replace("\r", "");
        break;
      }
      wifiFile.close();
      /*
      Serial.print("SSID: ");
      Serial.println(WifiSSID);
      Serial.print("PSK: ");
      Serial.println(WifiPSK);
      //GO.lcd.print("SSID: '");
      //GO.lcd.print(WifiSSID);
      //GO.lcd.println("'");
      //GO.lcd.print("PSK: '");
      //GO.lcd.print(WifiPSK);
      //GO.lcd.println("'");
      Serial.println();

      //delete old wifi Credentials
      WiFi.disconnect();

      WiFi.begin(WifiSSID.c_str(), WifiPSK.c_str());
      Serial.print("Connecting Wifi");
      //GO.lcd.print("Connecting Wifi");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        //GO.lcd.print(".");
      }
      */
      //GO.lcd.println(" ");
      Serial.println();
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      //GO.lcd.println("IP address: ");
      //GO.lcd.println(WiFi.localIP());

      //GO.lcd.println(" ");
      Serial.println();

      if (MDNS.begin("wifiserial")) {
        Serial.println("MDNS responder started");
        //GO.lcd.println("MDNS responder started");
        Serial.println("You can connect via http://wifiserial.local");
        //GO.lcd.println("You can connect via http://odroidgo.local");
      }
      break;
    }
    //GO.update();
  }

  //GO.lcd.println(" ");
  Serial.println();

  web_server.on("/", []() {
    handleRoot();
  });

  web_server.onNotFound(handleRoot);

  web_server.on("/fupload",  HTTP_POST, []() {
    web_server.send(200);
  }, handleFileUpload);

  web_server.on("/deleteConfirm", deleteConfirm);
  web_server.on("/doDelete", doDelete);
  web_server.on("/mkdir", doMkdir);
  web_server.begin();
  Serial.println("HTTP web_server started");
  //GO.lcd.println("HTTP web_server started");

  //GO.lcd.println();
  Serial.println();

  Serial.println("Initialization done.");
  //GO.lcd.println("Initializing done.");
}
#endif

#if 0
/***************************************************************************************
* Function name:          //M5Screen2bmp
* Description:            Dump the screen to a bmp image File
* Image file format:      .bmp
* return value:           true:  succesfully wrote screen to file
*                         false: unabel to open file for writing
* example for screen capture onto SD-Card: 
*                         //M5Screen2bmp(SD, "/screen.bmp");
* inspired by: https://stackoverflow.com/a/58395323
***************************************************************************************/
bool //M5Screen2bmp(fs::FS &fs, const char * path){
  // Open file for writing
  // The existing image file will be replaced
  File file = fs.open(path, FILE_WRITE);
  if(file){
    // M5Stack:      TFT_WIDTH = 240 / TFT_HEIGHT = 320
    // M5StickC:     TFT_WIDTH =  80 / TFT_HEIGHT = 160
    // M5StickCplus: TFT_WIDTH =  135 / TFT_HEIGHT = 240
    int image_height = 240;//M5.Lcd.height();
    int image_width = 320;//M5.Lcd.width();
    // horizontal line must be a multiple of 4 bytes long
    // add padding to fill lines with 0
    const uint pad=(4-(3*image_width)%4)%4;
    // header size is 54 bytes:
    //    File header = 14 bytes
    //    Info header = 40 bytes
    uint filesize=54+(3*image_width+pad)*image_height; 
    unsigned char header[54] = { 
      'B','M',  // BMP signature (Windows 3.1x, 95, NT, …)
      0,0,0,0,  // image file size in bytes
      0,0,0,0,  // reserved
      54,0,0,0, // start of pixel array
      40,0,0,0, // info header size
      0,0,0,0,  // image width
      0,0,0,0,  // image height
      1,0,      // number of color planes
      24,0,     // bits per pixel
      0,0,0,0,  // compression
      0,0,0,0,  // image size (can be 0 for uncompressed images)
      0,0,0,0,  // horizontal resolution (dpm)
      0,0,0,0,  // vertical resolution (dpm)
      0,0,0,0,  // colors in color table (0 = none)
      0,0,0,0 };// important color count (0 = all colors are important)
    // fill filesize, width and heigth in the header array
    for(uint i=0; i<4; i++) {
        header[ 2+i] = (char)((filesize>>(8*i))&255);
        header[18+i] = (char)((image_width   >>(8*i))&255);
        header[22+i] = (char)((image_height  >>(8*i))&255);
    }
    // write the header to the file
    file.write(header, 54);
    
    // To keep the required memory low, the image is captured line by line
    unsigned char line_data[image_width*3+pad];
    // initialize padded pixel with 0 
    for(int i=(image_width-1)*3; i<(image_width*3+pad); i++){
      line_data[i]=0;
    }
    // The coordinate origin of a BMP image is at the bottom left.
    // Therefore, the image must be read from bottom to top.
    for(int y=image_height; y>0; y--){
      // get one line of the screen content
      M5.Lcd.readRectRGB(0, y-1, image_width, 1, line_data);
      // BMP color order is: Blue, Green, Red
      // return values from readRectRGB is: Red, Green, Blue
      // therefore: R und B need to be swapped
      for(int x=0; x<image_width; x++){
        unsigned char r_buff = line_data[x*3];
        line_data[x*3] = line_data[x*3+2];
        line_data[x*3+2] = r_buff;
      }
      // write the line to the file
      file.write(line_data, (image_width*3)+pad);
    }
    file.close();
    return true;
  }
  return false;
}

#if 0
#define BitImageSize (320 * 240 *3) //画面キャプチャーイメージサイズ
#define PixelPerMeter (320 / 4 * 100) //画面解像度
//Function entry
void Screen_Capture_BMP(char *file_name)
{
  uint8_t FrameBuffer[320*3];
  File fp;
  int x,y,i;
  const uint16_t BMP_header[14 + 40] = //BMPファイルヘッダー定義
  { //File header 14 bytes
    'B','M', //uint16_t bfType
    (BitImageSize & 0xff),((BitImageSize>>8) & 0xff),((BitImageSize>>16) & 0xff),0, //uint32_t bfSize
    0,0, //uint16_t bfReserved1
    0,0, //uint16_t bfReserved2
    14+40,0,0,0,//uint32_t bfOffBits
    //Information header 40 bytes
    40,0,0,0, //uint32_t biSize
    (320 & 0xff),((320 >> 8)& 0xff),0,0, //int32_t biWidth
    240,0,0,0, //int32_t biHeight
    1,0, //uint16_t biPlanes
    24,0, //uint16_t biBitCount
    0,0,0,0, //uint32_t biCompression
    (BitImageSize & 0xff),((BitImageSize>>8) & 0xff),((BitImageSize>>16) & 0xff),0, //uint32_t biSizeImage
    (PixelPerMeter & 0xff),((PixelPerMeter>>8) & 0xff),0,0, //int32_t biXPelsPerMeter
    (PixelPerMeter & 0xff),((PixelPerMeter>>8) & 0xff),0,0, //int32_t biYPelsPerMeter
    0,0,0,0, //uint32_t biClrUsed
    0,0,0,0 
  }; //uint32_t biClrImportant

  Serial.printf("Start %s\n", file_name);
  M5.Lcd.setBrightness(10); //バックライトを暗くする

  if (!SD.begin(4)) {
    M5.Lcd.println("NO SD CARD.");
    M5.Lcd.setBrightness(200);
    while (1) ;
  }
  fp = SD.open(file_name, FILE_WRITE);
  if(!fp){
    M5.Lcd.println("File open error.");
    M5.Lcd.setBrightness(200);
    while (1) ;
  }
  for(i=0;i<(14+40);i++) fp.write(BMP_header[i]); //BMPファイルヘッダーの出力
  for(y=239;y>=0;y--){ //画面の下から上へスキャン
    M5.Lcd.readRectRGB(0, y, 320, 1, FrameBuffer); //１ライン分の画面データの取得
    for(x=0;x<320*3;x+=3){ //１ライン分のデータ出力
      fp.write(FrameBuffer[x+2]); //Blue
      fp.write(FrameBuffer[x+1]); //Green
      fp.write(FrameBuffer[x]); //Red
    }
  }
  fp.close();
  M5.Lcd.setBrightness(200); //バックライトの明るさを戻す
  Serial.printf("end %s\n", file_name);
}
#endif
#endif
/*
void loop(void) {
  //GO.update();
  web_server.handleClient();
}
*/
/*
WEB Serial Monitor
*/
#if 0

#define             FRMW_VERSION         "1.2236"
#define             PRGM_VERSION         "1.0"
#define             COMMAND_GET_SETTINGS          100
#define             COMMAND_GET_INFO              101
#define             COMMAND_REBOOT                998


extern const char   html_template[];
extern const char   js_main[];
extern const char   css_main[];



WebServer      web_serial_server(80);
WiFiClient          wifiClient;
WebSocketsServer    webSocket = WebSocketsServer(8080);

struct settings {
  char pversion[8];
  char ssid[32];
  char password[64];
  unsigned long baud;
} gateway_settings = {};

bool setup_mode = false;
int  connected_wifi_clients = 0;

void startAP(bool new_device) {

  WiFi.mode(WIFI_AP);
  WiFi.softAP("MrDIY Wireless Serial", "mrdiy.ca");
  setup_mode = true;
}

void sendDeviceMessage(String msg) {

  Serial.print(msg);
  Serial.print("\r\n");
}

void getDeviceMessages() {

  char buff[300];
  int state;
  int i;
  while (Serial.available() > 0) {
    i = Serial.readBytesUntil('\n', buff, sizeof(buff) - 1);
    buff[i] = '\0';
    sendRawDataOverSocket(buff, sizeof(buff));
  }
  i = 0;
}

void startServer() {

  web_serial_server.on("/",            handleMain);
  web_serial_server.on("/main.js",     handleMainJS);
  web_serial_server.on("/main.css",    handleMainCSS);
  web_serial_server.on("/update",      handleUpdate);
  web_serial_server.onNotFound(handle_NotFound);
  web_serial_server.on("/favicon.ico", handle_NotFound);
  web_serial_server.begin();
}

void handleMain() {

  if (setup_mode == true) {
    handleConfig();
    return;
  }
  web_serial_server.sendHeader("Cache-Control", "public, max-age=604800, immutable");
  web_serial_server.send_P(200, "text/html", html_template );
}

void handleUpdate() {

  long baud = web_serial_server.arg("baud").toInt();
  //gateway_settings.baud = baud;
  //EEPROM.put(0, gateway_settings);
  //EEPROM.commit();
  Serial.begin(baud);
  web_serial_server.send(200, "text/html", "1" );
}

void handleMainJS() {
  web_serial_server.sendHeader("Cache-Control", "public, max-age=604800, immutable");
  web_serial_server.send_P(200, "text/javascript", js_main);
}

void handleMainCSS() {

  web_serial_server.sendHeader("Cache-Control", "public, max-age=604800, immutable");
  web_serial_server.send_P(200, "text/css", css_main);
}

void handle_NotFound() {

  web_serial_server.send(404,   "text/html", "<html><body><p>404 Error</p></body></html>" );
}

void handleConfig() {

  if (web_serial_server.method() == HTTP_POST) {
    //strncpy(gateway_settings.ssid,              web_serial_server.arg("ssid").c_str(),             sizeof(gateway_settings.ssid) );
    //strncpy(gateway_settings.password,          web_serial_server.arg("password").c_str(),         sizeof(gateway_settings.password) );
    //gateway_settings.ssid[web_serial_server.arg("ssid").length()]  = gateway_settings.password[web_serial_server.arg("password").length()]  =  0;  // string terminate
    //strncpy(gateway_settings.pversion, PRGM_VERSION , sizeof(PRGM_VERSION) );
    //EEPROM.put(0, gateway_settings);
    //EEPROM.commit();
    String s = "<!DOCTYPE html><html lang='en'><head><meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'/>";
    s += "<meta content='text/html;charset=utf-8' http-equiv='Content-Type'>";
    s += "<title>Wireless Serial - MrDIY.ca</title>";
    s += "  <link rel='icon' type='image/png' sizes='16x16' href='data:image/x-icon;base64,AAABAAEAEBAAAAEAIABoBAAAFgAAACgAAAAQAAAAIAAAAAEAIAAAAAAAQAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIdUOc6IVTvpiFU654hVOuaIVTvkiVU74IdUOsqFUDaMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACIVDvli1c9/4pWPf+KVj3/ilY9/4pXPf+LVz3/jVk+/4lWPPaBSS5pAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgk064oVRPf+EUDz/hFA8/4RQPP98Rz3/fUk8/4VRPP+KVzz/jVk+/4ROMXwAAAAA/8s1///LNf//yzX//8s1///KM///yjL//8oy///KMv//yjL//8ky//C4Nf+wfjr/f0o8/4pWPP+LVz3/AAAAAP/FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xjP//8wy/76LOf+DTzz/jFg9/4ZTOLX/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xzP/f0s8/4pWPP+IVTvj/8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8oy/6JwO/+HUzz/ilY88//FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///KMv+fbTv/h1Q8/4pWPPP/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///GM///xTP/fUg8/4pWPP+JVjri/8Yz///GM///xjP//8Yz///GM///xjP//8Yz///GM///xjP//8Yz///HM///yzL/sX46/4RRPP+MWD3/h1I4sf/GMvT/xjL0/8Yy9P/FMvP4wDP99740//e+NP/3vjT/9740//O6NP/dpzf/m2k7/4JOPP+KVjz/iVY8/wAAAAAAAAAAAAAAAAAAAAAAAAAAd0A74X5IPf98Rzz/fEc8/3xHPP99SDz/gEw8/4dTPP+LVz3/jFg9/4FKL2sAAAAAAAAAAAAAAAAAAAAAAAAAAIhVOuaMWD7/i1c9/4tXPf+LVz3/i1c9/4xYPf+OWT//iFQ75XxDJ0gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACHUji2iFQ6zohTOc2HVDnMh1M5yodUOsaGUjerf0YqVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//8AAPAPAADwAwAA8AEAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAA8AEAAPADAADwDwAA//8AAA=='/>";
    s += "<meta content='utf-8' http-equiv='encoding'>";
    s += "<style>*,::after,::before{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans',sans-serif,'Apple Color Emoji','Segoe UI Emoji','Segoe UI Symbol','Noto Color Emoji';font-size:1rem;font-weight:400;line-height:1.5;color:#212529;background-color:#fff;-webkit-text-size-adjust:100%;-webkit-tap-highlight-color:transparent}a{color:#0d6efd;text-decoration:underline}a:hover{color:#0a58ca}::-moz-focus-inner{padding:0;border-style:none}.container{width:100%;padding-right:var(--bs-gutter-x,.75rem);padding-left:var(--bs-gutter-x,.75rem);margin-right:auto;margin-left:auto}@media (min-width:576px){.container{max-width:540px}}@media (min-width:768px){.container{max-width:720px}}@media (min-width:992px){.container{max-width:960px}}@media (min-width:1200px){.container{max-width:1140px}}@media (min-width:1400px){.container{max-width:1320px}}.row{--bs-gutter-x:1.5rem;--bs-gutter-y:0;display:flex;flex-wrap:wrap;margin-top:calc(var(--bs-gutter-y) * -1);margin-right:calc(var(--bs-gutter-x)/ -2);margin-left:calc(var(--bs-gutter-x)/ -2)}.row>*{flex-shrink:0;width:100%;max-width:100%;padding-right:calc(var(--bs-gutter-x)/ 2);padding-left:calc(var(--bs-gutter-x)/ 2);margin-top:var(--bs-gutter-y)}@media (min-width:768px){.col-md-12{flex:0 0 auto;width:100%}}.d-flex{display:flex!important}.d-inline-flex{display:inline-flex!important}.border-bottom{border-bottom:1px solid #dee2e6!important}.flex-column{flex-direction:column!important}.justify-content-between{justify-content:space-between!important}.align-items-center{align-items:center!important}.mt-2{margin-top:.5rem!important}.mb-4{margin-bottom:1.5rem!important}.py-3{padding-top:1rem!important;padding-bottom:1rem!important}.pb-3{padding-bottom:1rem!important}.fs-4{font-size:calc(1.275rem + .3vw)!important}.text-decoration-none{text-decoration:none!important}.text-dark{color:#212529!important}@media (min-width:768px){.flex-md-row{flex-direction:row!important}.mt-md-0{margin-top:0!important}.ms-md-auto{margin-left:auto!important}}@media (min-width:1200px){.fs-4{font-size:1.5rem!important}}body{display:flex;flex-wrap:nowrap;height:100vh;height:-webkit-fill-available;overflow-x:auto}body>*{flex-shrink:0;min-height:-webkit-fill-available}a{color:#3d568a!important}.logo{padding-left:40px;background-size:40px 18px;background-image:url(data:image/svg+xml,%3Csvg width='469' height='197' viewBox='0 0 469 197' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Crect width='469' height='197' fill='%23F2F2F2'/%3E%3Crect width='469' height='197' fill='white'/%3E%3Cpath d='M127.125 196C158.396 196 184.026 186.794 204.016 168.382C224.005 149.97 234 126.212 234 97.1091C234 68.0061 224.104 44.5455 204.312 26.7273C184.521 8.90909 158.792 0 127.125 0H63V196H127.125Z' fill='%233D568A'/%3E%3Cpath d='M0 156V40H127.837C148.103 40 164.197 45.1122 176.118 55.3365C188.039 65.5609 194 79.5962 194 97.4423C194 115.103 187.94 129.324 175.82 140.106C163.501 150.702 147.507 156 127.837 156H0Z' fill='%2332C5FF'/%3E%3Cpath d='M54.8659 136V96.0774L73.1454 119.714H77.9906L96.2701 96.0774V136H111.136V60H106.291L75.568 100.488L44.8452 60H40V136H54.8659Z' fill='white'/%3E%3Cpath d='M135.241 136V108.011C135.241 103.251 136.554 99.6253 139.181 97.1324C141.808 94.6394 145.416 93.393 150.004 93.393H154V79.9084C152.594 79.4551 150.966 79.2285 149.116 79.2285C142.826 79.2285 137.794 81.7215 134.02 86.7073V79.9084H120.256V136H135.241Z' fill='white'/%3E%3Cpath d='M284 98V1H244V98H284Z' fill='%233D568A'/%3E%3Cpath d='M284 197V98H244V197H284Z' fill='%2332C5FF'/%3E%3Cpath d='M401.669 119.703L469 1H424.805L381.5 78.7506L338.492 1H294L361.627 120L401.669 119.703Z' fill='%2332C5FF'/%3E%3Cpath d='M402 197V119.261L381.852 78L362 119.56V197H402Z' fill='%233D568A'/%3E%3C/svg%3E);background-repeat:no-repeat}</style>";
    s += "</head><body>";
    s += "<div class='container py-3'>";
    s += "<header>";
    s += "  <div class='d-flex flex-column flex-md-row align-items-center pb-3 mb-4 border-bottom' style='border-bottom:1px solid #32C5FF!important;'>";
    s += "    <a href='/' class='d-flex align-items-center text-dark text-decoration-none'><span class='fs-4 logo' style='height:19px;width:66px;'></span></a><span class='fs-4'>Wi-Fi Setup</span>";
    s += "    <nav class='d-inline-flex mt-2 mt-md-0 ms-md-auto'>";
    s += "    </nav>";
    s += "  </div>";
    s += "</header>";
    s += "<body><main>";
    s += "<div class='row justify-content-between'>";
    s += "<div class='col-md-12' style='text-align: center;'>";
    s += "<br /><h4 style='font-size:1.5rem;margin:0;'>Saved!</h4><br /><p>Please restart the device.</p>";
    s += "</div>";
    s += "</div></main></body></html>";
    web_serial_server.send(200, "text/html", s );

  } else {

    uint32_t realSize = 8000;//ESP.getFlashChipRealSize();
    uint32_t ideSize = 4000;//ESP.getFlashChipSize();
    String s = "<!DOCTYPE html><html lang='en'><head><meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'/>";
    s += "<meta content='text/html;charset=utf-8' http-equiv='Content-Type'>";
    s += "<title>Wireless Serial - MrDIY.ca</title>";
    s += "  <link rel='icon' type='image/png' sizes='16x16' href='data:image/x-icon;base64,AAABAAEAEBAAAAEAIABoBAAAFgAAACgAAAAQAAAAIAAAAAEAIAAAAAAAQAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIdUOc6IVTvpiFU654hVOuaIVTvkiVU74IdUOsqFUDaMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACIVDvli1c9/4pWPf+KVj3/ilY9/4pXPf+LVz3/jVk+/4lWPPaBSS5pAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgk064oVRPf+EUDz/hFA8/4RQPP98Rz3/fUk8/4VRPP+KVzz/jVk+/4ROMXwAAAAA/8s1///LNf//yzX//8s1///KM///yjL//8oy///KMv//yjL//8ky//C4Nf+wfjr/f0o8/4pWPP+LVz3/AAAAAP/FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xjP//8wy/76LOf+DTzz/jFg9/4ZTOLX/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xzP/f0s8/4pWPP+IVTvj/8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8oy/6JwO/+HUzz/ilY88//FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///KMv+fbTv/h1Q8/4pWPPP/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///GM///xTP/fUg8/4pWPP+JVjri/8Yz///GM///xjP//8Yz///GM///xjP//8Yz///GM///xjP//8Yz///HM///yzL/sX46/4RRPP+MWD3/h1I4sf/GMvT/xjL0/8Yy9P/FMvP4wDP99740//e+NP/3vjT/9740//O6NP/dpzf/m2k7/4JOPP+KVjz/iVY8/wAAAAAAAAAAAAAAAAAAAAAAAAAAd0A74X5IPf98Rzz/fEc8/3xHPP99SDz/gEw8/4dTPP+LVz3/jFg9/4FKL2sAAAAAAAAAAAAAAAAAAAAAAAAAAIhVOuaMWD7/i1c9/4tXPf+LVz3/i1c9/4xYPf+OWT//iFQ75XxDJ0gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACHUji2iFQ6zohTOc2HVDnMh1M5yodUOsaGUjerf0YqVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//8AAPAPAADwAwAA8AEAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAA8AEAAPADAADwDwAA//8AAA=='/>";
    s += "<meta content='utf-8' http-equiv='encoding'>";
    s += "<style>*,::after,::before{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans',sans-serif,'Apple Color Emoji','Segoe UI Emoji','Segoe UI Symbol','Noto Color Emoji';font-size:1rem;font-weight:400;line-height:1.5;color:#212529;background-color:#fff;-webkit-text-size-adjust:100%;-webkit-tap-highlight-color:transparent}a{color:#0d6efd;text-decoration:underline}a:hover{color:#0a58ca}label{display:inline-block}button{border-radius:0}button:focus:not(:focus-visible){outline:0}button,input{margin:0;font-family:inherit;font-size:inherit;line-height:inherit}button{text-transform:none}[type=submit],button{-webkit-appearance:button}::-moz-focus-inner{padding:0;border-style:none}.container{width:100%;padding-right:var(--bs-gutter-x,.75rem);padding-left:var(--bs-gutter-x,.75rem);margin-right:auto;margin-left:auto}@media (min-width:576px){.container{max-width:540px}}@media (min-width:768px){.container{max-width:720px}}@media (min-width:992px){.container{max-width:960px}}@media (min-width:1200px){.container{max-width:1140px}}@media (min-width:1400px){.container{max-width:1320px}}.row{--bs-gutter-x:1.5rem;--bs-gutter-y:0;display:flex;flex-wrap:wrap;margin-top:calc(var(--bs-gutter-y) * -1);margin-right:calc(var(--bs-gutter-x)/ -2);margin-left:calc(var(--bs-gutter-x)/ -2)}.row>*{flex-shrink:0;width:100%;max-width:100%;padding-right:calc(var(--bs-gutter-x)/ 2);padding-left:calc(var(--bs-gutter-x)/ 2);margin-top:var(--bs-gutter-y)}.col-12{flex:0 0 auto;width:100%}@media (min-width:768px){.col-md-12{flex:0 0 auto;width:100%}}.form-label{margin-bottom:.5rem}.form-control{display:block;width:100%;padding:.375rem .75rem;font-size:1rem;font-weight:400;line-height:1.5;color:#212529;background-color:#fff;background-clip:padding-box;border:1px solid #ced4da;-webkit-appearance:none;-moz-appearance:none;appearance:none;border-radius:.25rem;transition:border-color .15s ease-in-out,box-shadow .15s ease-in-out}@media (prefers-reduced-motion:reduce){.form-control{transition:none}}.form-control:focus{color:#212529;background-color:#fff;border-color:#86b7fe;outline:0;box-shadow:0 0 0 .25rem rgba(13,110,253,.25)}.form-control::-moz-placeholder{color:#6c757d;opacity:1}.form-control::placeholder{color:#6c757d;opacity:1}.form-control:disabled{background-color:#e9ecef;opacity:1}.form-floating{position:relative}.btn{display:inline-block;font-weight:400;line-height:1.5;color:#212529;text-align:center;text-decoration:none;vertical-align:middle;cursor:pointer;-webkit-user-select:none;-moz-user-select:none;user-select:none;background-color:transparent;border:1px solid transparent;padding:.375rem .75rem;font-size:1rem;border-radius:.25rem;transition:color .15s ease-in-out,background-color .15s ease-in-out,border-color .15s ease-in-out,box-shadow .15s ease-in-out}@media (prefers-reduced-motion:reduce){.btn{transition:none}}.btn:hover{color:#212529}.btn:focus{outline:0;box-shadow:0 0 0 .25rem rgba(13,110,253,.25)}.btn:disabled{pointer-events:none;opacity:.65}.btn-primary{color:#fff;background-color:#0d6efd;border-color:#0d6efd}.btn-primary:hover{color:#fff;background-color:#0b5ed7;border-color:#0a58ca}.btn-primary:focus{color:#fff;background-color:#0b5ed7;border-color:#0a58ca;box-shadow:0 0 0 .25rem rgba(49,132,253,.5)}.btn-primary:active{color:#fff;background-color:#0a58ca;border-color:#0a53be}.btn-primary:active:focus{box-shadow:0 0 0 .25rem rgba(49,132,253,.5)}.btn-primary:disabled{color:#fff;background-color:#0d6efd;border-color:#0d6efd}.btn-lg{padding:.5rem 1rem;font-size:1.25rem;border-radius:.3rem}.d-flex{display:flex!important}.d-inline-flex{display:inline-flex!important}.border-bottom{border-bottom:1px solid #dee2e6!important}.flex-column{flex-direction:column!important}.justify-content-between{justify-content:space-between!important}.align-items-center{align-items:center!important}.mt-2{margin-top:.5rem!important}.mt-3{margin-top:1rem!important}.mb-4{margin-bottom:1.5rem!important}.py-3{padding-top:1rem!important;padding-bottom:1rem!important}.pb-3{padding-bottom:1rem!important}.fs-4{font-size:calc(1.275rem + .3vw)!important}.text-decoration-none{text-decoration:none!important}.text-dark{color:#212529!important}@media (min-width:768px){.flex-md-row{flex-direction:row!important}.mt-md-0{margin-top:0!important}.ms-md-auto{margin-left:auto!important}}@media (min-width:1200px){.fs-4{font-size:1.5rem!important}}body{display:flex;flex-wrap:nowrap;height:100vh;height:-webkit-fill-available;overflow-x:auto}body>*{flex-shrink:0;min-height:-webkit-fill-available}a{color:#3d568a!important}.btn-primary{background-color:#3d568a!important}.logo{padding-left:40px;background-size:40px 18px;background-image:url(\"data:image/svg+xml,%3Csvg width='469' height='197' viewBox='0 0 469 197' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Crect width='469' height='197' fill='%23F2F2F2'/%3E%3Crect width='469' height='197' fill='white'/%3E%3Cpath d='M127.125 196C158.396 196 184.026 186.794 204.016 168.382C224.005 149.97 234 126.212 234 97.1091C234 68.0061 224.104 44.5455 204.312 26.7273C184.521 8.90909 158.792 0 127.125 0H63V196H127.125Z' fill='%233D568A'/%3E%3Cpath d='M0 156V40H127.837C148.103 40 164.197 45.1122 176.118 55.3365C188.039 65.5609 194 79.5962 194 97.4423C194 115.103 187.94 129.324 175.82 140.106C163.501 150.702 147.507 156 127.837 156H0Z' fill='%2332C5FF'/%3E%3Cpath d='M54.8659 136V96.0774L73.1454 119.714H77.9906L96.2701 96.0774V136H111.136V60H106.291L75.568 100.488L44.8452 60H40V136H54.8659Z' fill='white'/%3E%3Cpath d='M135.241 136V108.011C135.241 103.251 136.554 99.6253 139.181 97.1324C141.808 94.6394 145.416 93.393 150.004 93.393H154V79.9084C152.594 79.4551 150.966 79.2285 149.116 79.2285C142.826 79.2285 137.794 81.7215 134.02 86.7073V79.9084H120.256V136H135.241Z' fill='white'/%3E%3Cpath d='M284 98V1H244V98H284Z' fill='%233D568A'/%3E%3Cpath d='M284 197V98H244V197H284Z' fill='%2332C5FF'/%3E%3Cpath d='M401.669 119.703L469 1H424.805L381.5 78.7506L338.492 1H294L361.627 120L401.669 119.703Z' fill='%2332C5FF'/%3E%3Cpath d='M402 197V119.261L381.852 78L362 119.56V197H402Z' fill='%233D568A'/%3E%3C/svg%3E\");background-repeat:no-repeat}.alert-danger{position:relative;padding:1rem 1rem;margin-bottom:1rem;border-radius:.25rem;color:#842029;background-color:#f8d7da;border-color:#f5c2c7;}</style>";
    s += "</head><body>";
    s += "<div class='container py-3'>";
    s += "<header>";
    s += "  <div class='d-flex flex-column flex-md-row align-items-center pb-3 mb-4 border-bottom' style='border-bottom:1px solid #32C5FF!important;'>";
    s += "    <a href='/' class='d-flex align-items-center text-dark text-decoration-none'><span class='fs-4 logo' style='height:19px;width:66px;'></span></a><span class='fs-4'>Configuration</span>";
    s += "    <nav class='d-inline-flex mt-2 mt-md-0 ms-md-auto'>";
    s += "    </nav>";
    s += "  </div>";
    s += "</header>";
    s += "<body><main>";
    s += "<div class='row justify-content-between'>";
    if (ideSize != realSize) s += "<div class='alert-danger'>Your flash size (" + String(ideSize) + ") is configured incorrectly. It should be " + String(realSize) + ".</div>";
    s += "<div class='col-md-12'>";
    s += "<form action='/' method='post'>";
    s += "    <div class='col-12 mt-3'><label class='form-label'>Wi-Fi Name</label><input type='text' name='ssid' class='form-control' value='" + String("SSID") + "'></div>"; //String(gateway_settings.ssid)
    s += "    <div class='col-12 mt-3'><label class='form-label'>Password</label><input type='password' name='password' class='form-control' value='' autocomplete='off'></div>";
    s += "    <div class='form-floating'><br/><button class='btn btn-primary btn-lg' type='submit'>Save</button><br /><br /><br /></div>";
    s += " </form>";
    s += "</div>";
    s += "</div></main></body></html>";
    web_serial_server.send(200, "text/html", s );
  }
}

/* ############################ tools ############################################# */


void sendRawDataOverSocket(const char * msgc, int len) {

  if ( strlen(msgc) == 0 or len == 0  ) return;
  StaticJsonDocument<400> console_data;
  console_data["type"] = "console";
  console_data["content"] = msgc;
  char   b[400];
  size_t lenn = serializeJson(console_data, b);  // serialize to buffer
  webSocket.broadcastTXT(b, lenn);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
    case WStype_TEXT: {
        StaticJsonDocument<300> socket_data;
        DeserializationError error = deserializeJson(socket_data, payload);
        if (error) {
          StaticJsonDocument<70> error_data;
          error_data["type"] = "error";
          error_data["content"] = "error parsing the last message";
          char   b[70]; // create temp buffer
          size_t len = serializeJson(error_data, b);  // serialize to buffer
          webSocket.sendTXT(num, b);
          break;
        }
        if ( socket_data["action"] == "command") {
          if ( socket_data["content"] == COMMAND_GET_INFO)             sendInfo(num);
          else if ( socket_data["content"] == COMMAND_REBOOT)               reboot();
          else {
            StaticJsonDocument<50> error_data;
            error_data["type"] = "error";
            error_data["content"] = "unknown command";
            char   b[50];
            size_t len = serializeJson(error_data, b);  // serialize to buffer
            webSocket.sendTXT(num, b);
          }
        } else if (socket_data["action"] == "console") {
          sendDeviceMessage(socket_data["content"]);
        }
      }
      break;
    default:
      break;
  }
}

void sendInfo(uint8_t num) {

  byte mac_address[6];
  WiFi.macAddress(mac_address);
  StaticJsonDocument<150> info_data;
  info_data["type"] = "info";
  info_data["version"] = PRGM_VERSION;//gateway_settings.pversion;
  info_data["wifi"] = String(WiFi.RSSI());
  info_data["ip_address"] = WiFi.localIP().toString();
  info_data["mac_address"] = WiFi.macAddress();
  info_data["version"] = FRMW_VERSION;
  info_data["baud"] = 115200;//gateway_settings.baud;
  char   b[150];
  size_t len = serializeJson(info_data, b);  // serialize to buffer
  if (num != 255) webSocket.sendTXT(num, b);
  else webSocket.broadcastTXT(b, len);
}

void sendErrorOverSocket(uint8_t num, const char * msg) {

  StaticJsonDocument<100> error_message;
  error_message["type"] = "error";
  error_message["msg"] = msg;
  char   b[100];
  size_t len = serializeJson(error_message, b);  // serialize to buffer
  if (num != 255) webSocket.sendTXT(num, b);
  else webSocket.broadcastTXT(b, len);
}


void reboot() {

  ESP.restart();
}

/* ############################ setup ########################################### */

void web_serial_setup() {

  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);  // on
  //EEPROM.begin(sizeof(struct settings) );
  //delay(2000);
  //EEPROM.get( 0, gateway_settings );
  //if ( String(gateway_settings.pversion) != PRGM_VERSION )
  //  memset(&gateway_settings, 0, sizeof(settings));
  //unsigned long serial_baud = gateway_settings.baud;
  //if ( !(serial_baud > 0 && serial_baud < 115200) ) serial_baud = 115200;
  //Serial.begin( serial_baud );
  //delay(500);
  //WiFi.mode(WIFI_STA);
  //WiFi.begin(gateway_settings.ssid, gateway_settings.password);
  //WiFi.setAutoReconnect(true);
  //WiFi.persistent(true);
  /*
  byte tries = 1;
  if ( String(gateway_settings.ssid) != "" && String(gateway_settings.password) != ""  ) {
    while (WiFi.status() != WL_CONNECTED ) {
      if (tries++ > 9 ) {
        startAP(false);
        break;
      }
      delay(1000);
    }
  } else {
    startAP(true);
  }
  */
  startServer();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  //ArduinoOTA.setHostname("WiFiSerial");
  //ArduinoOTA.begin();
  //digitalWrite(LED_BUILTIN, HIGH);
}

/* ############################ loop ############################################# */

void web_serial_loop() {

  web_serial_server.handleClient();
  webSocket.loop();
  getDeviceMessages();
  //ArduinoOTA.handle();
  if ( setup_mode && connected_wifi_clients != WiFi.softAPgetStationNum() ) connected_wifi_clients = WiFi.softAPgetStationNum();
}
#endif


File printFile;
String buffer;
boolean SDfound;


void file_send() {
  if (SDfound == 0) {
    if (!SD.begin(53)) {
      Serial.print("The SD card cannot be found");
      //while(1);
      return;
    }
  }
  SDfound = 1;
  printFile = SD.open("/send_file.txt");

  if (!printFile) {
    Serial.print("The text file cannot be opened");
    //while(1);
    return;
  }

  while (printFile.available()) {
    unsigned long currentMillis = millis();
    buffer = printFile.readStringUntil('\n') + '\n' +'\r';
    while(printFile.available()) {
      buffer += printFile.readStringUntil('\n') + '\n' + '\r';
      if((buffer.indexOf("$GPVTG")) >= 0) {
        break;
      }
    }
    //buffer += printFile.readStringUntil('\n');
    Serial.print(buffer); //Printing for debugging purpose    
    MySerial.print(buffer); //Printing for debugging purpose 
    esp_task_wdt_reset();   // Added to repeatedly reset the Watch Dog Timer
    int timer_out_cnt = 0;
    while(1) {
      timer_out_cnt++;
      if(((currentMillis + 1000) <= millis()) || (timer_out_cnt > 500)) {
        break;
      }
      delay(2);
    }  
    //do some action here
  }

  printFile.close();
}