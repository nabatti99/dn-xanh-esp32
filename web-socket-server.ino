/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-websocket-server-sensor/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

// Define pins
#define DOOR_ECHO_PIN 23
#define SOUND_TRIGGER_PIN 5
#define SOUND_ECHO_PIN 18

// Constants
enum State {IDLE, OPENING_DOOR, COLLECTING_DATA, SERVER_PROCESSING, CLAIM_REWARD};
const String WASTE_TYPE = "RECYCLE";
const double SOUND_SPEED = 0.034;
const unsigned long TIMEOUT = 30000;

// Replace with your network credentials
const char* ssid = "minh_nguyenanh";
const char* password = "123456789";

// Set your Static IP config
IPAddress localIP(192, 168, 137, 20);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Global variables
State state = IDLE;
bool isConnectedToWifi = false;
uint32_t currentClientId = 0;

String stateToString(State state) {
  switch (state) {
    case IDLE:
      return "IDLE";

    case OPENING_DOOR:
      return "OPENING_DOOR";

    case COLLECTING_DATA:
      return "COLLECTING_DATA";

    case SERVER_PROCESSING:
      return "SERVER_PROCESSING";

    case CLAIM_REWARD:
      return "CLAIM_REWARD";
  }
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected");
  isConnectedToWifi = true;

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}

// Initialize WiFi
void initWiFi() {
  WiFi.disconnect(true);

  delay(200);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // // Dynamic IP
  // WiFi.mode(WIFI_STA);

  // Static IP
  if (!WiFi.config(localIP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi ...");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    Serial.print("Received message");
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      currentClientId = client->id();
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

void initDoor() {
  pinMode(DOOR_ECHO_PIN, INPUT);
}

void initSound() {
  pinMode(SOUND_TRIGGER_PIN, OUTPUT);
  pinMode(SOUND_ECHO_PIN, INPUT);
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  initWebSocket();
  initDoor();
  initSound();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hello World!");
  });

  // Start server
  server.begin();
}

void sendMessage(String message) {
  ws.text(currentClientId, message);
  Serial.println(message);
}

void sendError(String error) {
  JSONVar responseData;
  responseData["type"] = "ERROR";
  responseData["state"] = error;
  String jsonresponseData = JSON.stringify(responseData);
  sendMessage(jsonresponseData);
}

void setState(State newState) {
  state = newState;

  JSONVar responseData;
  responseData["type"] = "SET_STATE";
  responseData["state"] = stateToString(newState);
  String jsonresponseData = JSON.stringify(responseData);
  sendMessage(jsonresponseData);
}

bool checkDoor() {
  const bool isOpen = digitalRead(DOOR_ECHO_PIN);
  return isOpen;
}

double getHeight() {
  // Clears the trigPin
  digitalWrite(SOUND_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(SOUND_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SOUND_TRIGGER_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  unsigned long soundDuration = pulseIn(SOUND_ECHO_PIN, HIGH);
  
  // Calculate the distance
  double distanceCm = soundDuration * SOUND_SPEED/2;
  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distanceCm);

  return distanceCm;
}

void breakLoop() {
  // Limiting the number of web socket clients
  ws.cleanupClients();
  delay(1000);
}

void loop() {
  static unsigned long beginTime = 0;
  static double previousHeight = 0;

  // Verify wifi connection
  if (!isConnectedToWifi) return breakLoop();

  // Verify websocket connection
  if (currentClientId == 0) {
    Serial.println("Waiting for client connect...");
    state = IDLE;

    return breakLoop();
  } 
  
  bool isDoorOpened = checkDoor();
  double height = getHeight();

  // Send sensors data to monitor
  JSONVar sensorData;
  sensorData["type"] = "SENSORS_DATA";
  sensorData["isDoorOpened"] = String(isDoorOpened);
  sensorData["height"] = String(height);
  String jsonSensorData = JSON.stringify(sensorData);
  sendMessage(jsonSensorData);

  // Check door
  if (isDoorOpened && state != OPENING_DOOR) {
    setState(OPENING_DOOR);
    beginTime = millis();

    return breakLoop();
  }

  if (state == IDLE) {
    previousHeight = height;
  }

  if (state == OPENING_DOOR && !isDoorOpened) {
    setState(COLLECTING_DATA);
    beginTime = millis();

    return breakLoop();
  }

  if (state == COLLECTING_DATA) {
    double sumHeightDelta = 0;
    for (int i = 0; i < 5; ++i) {
      // Capture image
      // Calculate height
      height = getHeight();
      double heightDelta = abs(height - previousHeight);
      sumHeightDelta += heightDelta;
      delay(200);
    }
    double avgHeightDelta = sumHeightDelta / 5;

    // Send to server
    setState(SERVER_PROCESSING);

    JSONVar serverRequestData;
    serverRequestData["heightDelta"] = avgHeightDelta;
    String jsonServerRequestData = JSON.stringify(serverRequestData);

    // Receive prediction
    // Send to build QR

    JSONVar responseData;
    responseData["type"] = "BUILD_QR";
    responseData["height"] = avgHeightDelta;
    responseData["wasteType"] = WASTE_TYPE;
    responseData["predictWasteType"] = "RECYCLE";
    responseData["predictAccuracy"] = 0.82;
    String jsonresponseData = JSON.stringify(responseData);
    sendMessage(jsonresponseData);

    setState(CLAIM_REWARD);
  }

  if (state != IDLE) {
    // Check timeout
    unsigned long processingTimeConsumed = millis() - beginTime;
    if (processingTimeConsumed > TIMEOUT) {
      sendError("Timeout");
      setState(IDLE);

      return breakLoop();
    } 
  }

  return breakLoop();
}