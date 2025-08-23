#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Arduino_JSON.h>

#define STM32_SERIAL Serial

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

uint8_t uart_msg[6];
String message = "";
String sliderValue1 = "0";
String sliderValue2 = "0";
String sliderValue3 = "0";
String sliderValue4 = "0";
String sliderValue5 = "0";
String sliderValue6 = "0";
String sliderValue7 = "0";
String sliderValue8 = "0";


int kp_percentage;
int ki_percentage;
int kd_percentage;
int tau_percentage;
int roll_ref_percentage;
int pitch_ref_percentage;
int yaw_ref_percentage;
int z_ref_percentage;

const char* ssid = "ESP8266-Access-Point";
const char* password = "123456789";

JSONVar sliderValues;

String getSliderValues() {
  sliderValues["sliderValue1"] = sliderValue1;
  sliderValues["sliderValue2"] = sliderValue2;
  sliderValues["sliderValue3"] = sliderValue3;
  sliderValues["sliderValue4"] = sliderValue4;
  sliderValues["sliderValue5"] = sliderValue5;
  sliderValues["sliderValue6"] = sliderValue6;
  sliderValues["sliderValue7"] = sliderValue7;
  sliderValues["sliderValue8"] = sliderValue8;
  return JSON.stringify(sliderValues);
}

uint8_t crc8(uint8_t *data, uint8_t data_length) {
  uint8_t crc = 0xFF;
  uint8_t poly = 0x1D;
  while (data_length--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      crc = crc & 0x80 ? (crc << 1) ^ poly : crc << 1;
    }
  }
  return crc;
}

void initFS() {
  if (!LittleFS.begin()) {
    //Serial.println("Error mounting LittleFS");
  } else {
    //Serial.println("LittleFS mounted");
  }
}

void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  //Serial.print("AP IP: ");
  //Serial.println(WiFi.softAPIP());
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  static unsigned long lastSendTime = 0;
  const unsigned long throttleInterval = 50;
  
  if (millis() - lastSendTime < throttleInterval) return;
  lastSendTime = millis();

  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    
    if (message.indexOf("1s") >= 0 || message.indexOf("2s") >= 0 || 
        message.indexOf("3s") >= 0 || message.indexOf("4s") >= 0 ||
        message.indexOf("5s") >= 0 || message.indexOf("6s") >= 0 || 
        message.indexOf("7s") >= 0 || message.indexOf("8s") >= 0) {
      
      if (message.indexOf("1s") >= 0) {
        sliderValue1 = message.substring(2);
        kp_percentage = sliderValue1.toInt();
        uart_msg[0] = 0x01;
        memcpy(uart_msg + 1, (char*)&kp_percentage, 4);
      } 
      else if (message.indexOf("2s") >= 0) {
        sliderValue2 = message.substring(2);
        ki_percentage = sliderValue2.toInt();
        uart_msg[0] = 0x02;
        memcpy(uart_msg + 1, (char*)&ki_percentage, 4);
      }
      else if (message.indexOf("3s") >= 0) {
        sliderValue3 = message.substring(2);
        kd_percentage = sliderValue3.toInt();
        uart_msg[0] = 0x03;
        memcpy(uart_msg + 1, (char*)&kd_percentage, 4);
      }
      else if (message.indexOf("4s") >= 0) {
        sliderValue4 = message.substring(2);
        tau_percentage = sliderValue4.toInt();
        uart_msg[0] = 0x04;
        memcpy(uart_msg + 1, (char*)&tau_percentage, 4);
      }
      else if (message.indexOf("5s") >= 0) {
        sliderValue5 = message.substring(2);
        roll_ref_percentage = sliderValue5.toInt();
        uart_msg[0] = 0x05;
        memcpy(uart_msg + 1, (char*)&roll_ref_percentage, 4);
      }
      else if (message.indexOf("6s") >= 0) {
        sliderValue6 = message.substring(2);
        pitch_ref_percentage = sliderValue6.toInt();
        uart_msg[0] = 0x06;
        memcpy(uart_msg + 1, (char*)&pitch_ref_percentage, 4);
      }
      else if (message.indexOf("7s") >= 0) {
        sliderValue7 = message.substring(2);
        yaw_ref_percentage = sliderValue7.toInt();
        uart_msg[0] = 0x07;
        memcpy(uart_msg + 1, (char*)&yaw_ref_percentage, 4);
      }
      else if (message.indexOf("8s") >= 0) {
        sliderValue8 = message.substring(2);
        z_ref_percentage = sliderValue8.toInt();
        uart_msg[0] = 0x08;
        memcpy(uart_msg + 1, (char*)&z_ref_percentage, 4);
      }

      uart_msg[5] = crc8(uart_msg, 5);
      
      while (STM32_SERIAL.availableForWrite() < 6) delay(1);
      STM32_SERIAL.write(uart_msg, 6);
      STM32_SERIAL.flush();
      
      //Serial.println(getSliderValues());
      notifyClients(getSliderValues());
    }
    else if (strcmp((char*)data, "getValues") == 0) {
      notifyClients(getSliderValues());
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      //Serial.printf("Client #%u connected\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      //Serial.printf("Client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    default: break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  STM32_SERIAL.begin(57600);

  initFS();
  initWiFi();
  initWebSocket();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.begin();
}

void loop() {
  ws.cleanupClients();
}