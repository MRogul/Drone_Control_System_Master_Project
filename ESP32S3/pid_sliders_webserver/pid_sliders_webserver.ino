/* 
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-websocket-sliders/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>
//#include "..\wifi_credentials\wifi_credentials.h"

#define ESP32TX_TO_STM32RX 17
#define ESP32RX_TO_STM32TX 16

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object

AsyncWebSocket ws("/ws");

uint8_t uart_msg[6];

String message = "";
String sliderValue1 = "0";
String sliderValue2 = "0";
String sliderValue3 = "0";
String sliderValue4 = "0";

int kp_percentage;
int ki_percentage;
int kd_percentage;
int tau_percentage;

const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

//Json Variable to Hold Slider Values
JSONVar sliderValues;

//Get Slider Values
String getSliderValues() {
  sliderValues["sliderValue1"] = String(sliderValue1);
  sliderValues["sliderValue2"] = String(sliderValue2);
  sliderValues["sliderValue3"] = String(sliderValue3);
  sliderValues["sliderValue4"] = String(sliderValue4);

  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
}

// CRC8_SAE_J1850
// https://www.sunshine2k.de/coding/javascript/crc/crc_js.html
uint8_t crc8(uint8_t *data, uint8_t data_length) {
  uint8_t crc = 0xFF;  // initial value
  uint8_t crc_polynomial = 0x1D;

  while (data_length--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      // https://stackoverflow.com/questions/51752284/how-to-calculate-crc8-in-c
      // https://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
      crc = crc & 0x80 ? (crc << 1) ^ crc_polynomial : crc << 1;
    }
  }
  return crc;
}

// Initialize LittleFS
void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {
  //WiFi.mode(WIFI_STA);
  //WiFi.begin("Orange_Swiatlowod_AEA0", "Michal2000");

  WiFi.mode(WIFI_AP);  
  WiFi.softAP(ssid, password);  //Start HOTspot removing password will disable security

  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char *)data;
    if (message.indexOf("1s") >= 0) {
      sliderValue1 = message.substring(2);
      kp_percentage = map(sliderValue1.toInt(), 0, 100, 0, 100);
      uart_msg[0] = 0x01;  // msg ID
      memcpy(uart_msg + 1, static_cast<char *>(static_cast<void *>(&kp_percentage)), 4);
      uart_msg[5] = crc8(uart_msg, 5);
      Serial2.write(uart_msg, 6);
      // Serial1.write(crc8(uart_msg, 6));
      Serial.println(getSliderValues());
      Serial.printf("CRC: 0x%02x\n", uart_msg[5]);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      sliderValue2 = message.substring(2);
      ki_percentage = map(sliderValue2.toInt(), 0, 100, 0, 100);
      uart_msg[0] = 0x02;  // msg ID
      memcpy(uart_msg + 1, static_cast<char *>(static_cast<void *>(&ki_percentage)), 4);
      uart_msg[5] = crc8(uart_msg, 5);
      Serial2.write(uart_msg, 6);
      // Serial1.write(crc8(uart_msg, 6));
      Serial.println(getSliderValues());
      Serial.printf("CRC: 0x%02x\n", uart_msg[5]);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      kd_percentage = map(sliderValue3.toInt(), 0, 100, 0, 100);
      uart_msg[0] = 0x03;  // msg ID
      memcpy(uart_msg + 1, static_cast<char *>(static_cast<void *>(&kd_percentage)), 4);
      uart_msg[5] = crc8(uart_msg, 5);
      Serial2.write(uart_msg, 6);
      // Serial1.write(crc8(uart_msg, 6));
      Serial.println(getSliderValues());
      Serial.printf("CRC: 0x%02x\n", uart_msg[5]);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("4s") >= 0) {
      sliderValue4 = message.substring(2);
      tau_percentage = map(sliderValue4.toInt(), 0, 100, 0, 100);
      uart_msg[0] = 0x04;  // msg ID
      memcpy(uart_msg + 1, static_cast<char *>(static_cast<void *>(&tau_percentage)), 4);
      uart_msg[5] = crc8(uart_msg, 5);
      Serial2.write(uart_msg, 6);
      // Serial1.write(crc8(uart_msg, 6));
      Serial.println(getSliderValues());
      Serial.printf("CRC: 0x%02x\n", uart_msg[5]);
      notifyClients(getSliderValues());
    }
    if (strcmp((char *)data, "getValues") == 0) {
      notifyClients(getSliderValues());
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, ESP32RX_TO_STM32TX, ESP32TX_TO_STM32RX);  // to STM32
  initFS();
  initWiFi();

  initWebSocket();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  // Start server
  server.begin();
}

void loop() {
  ws.cleanupClients();
}
