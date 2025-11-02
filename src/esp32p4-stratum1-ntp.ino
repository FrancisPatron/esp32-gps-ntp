#include <WiFi.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// WiFi options
String ssid = "";
String password = "";
bool enableWiFi = false;

// Adjust for your esp32p4 board
HardwareSerial gpsSerial(2);
#define GPS_RX_PIN 20
#define GPS_TX_PIN 5
#define PPS_PIN 21 
#define GPS_BAUD 9600
#define NTP_PORT 123
#define TCP_PORT 2947

WebServer server(80);
WiFiUDP ntpUDP;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
TinyGPSPlus gps;

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

volatile unsigned long ppsTimeMicros = 0;
volatile unsigned long lastPPSMicros = 0;
volatile bool ppsReceived = false;

unsigned long gpsTimeSeconds = 0;
bool timeValid = false;
unsigned long lastPrintTime = 0;
unsigned long lastNetworkCheck = 0;

bool ethConnected = false;
bool wifiConnected = false;

void IRAM_ATTR ppsInterrupt() {
  unsigned long now = micros();
  if (now - lastPPSMicros > 500000) {
    ppsTimeMicros = now;
    ppsReceived = true;
    lastPPSMicros = now;
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-gps-ntp");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH Got IP: ");
      Serial.println(ETH.localIP());
      ethConnected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      ethConnected = false;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi Got IP: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi Disconnected");
      wifiConnected = false;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== ESP32-P4 GPS NTP Server ===");
  Serial.println("Initializing...\n");
  
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);
  Serial.println("PPS interrupt configured on GPIO " + String(PPS_PIN));
  
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started");
  
  WiFi.onEvent(WiFiEvent);
  
  Serial.println("\nTrying Ethernet...");
  if (!ETH.begin()) {
    Serial.println("ETH initialization failed");
  }
  delay(3000);
  
  if (!ethConnected) {
    Serial.println("Ethernet not connected, trying WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 30) {
      delay(500);
      Serial.print(".");
      timeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi Connected!");
      wifiConnected = true;
    } else {
      Serial.println("\nWiFi connection failed");
    }
  }
  
  if (!ethConnected && !wifiConnected) {
    Serial.println("\nNo network connection - continuing without network");
    Serial.println("GPS and PPS will still work for local timing");
  }
  
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
  
  ntpUDP.begin(NTP_PORT);
  tcpServer.begin();
  
  Serial.println("\nServices started:");
  Serial.println("- NTP server on port " + String(NTP_PORT));
  Serial.println("- TCP NMEA server on port " + String(TCP_PORT));
  Serial.println("- Web interface on port 80");
  if (ethConnected) {
    Serial.println("- Ethernet IP: " + ETH.localIP().toString());
  }
  if (wifiConnected) {
    Serial.println("- WiFi IP: " + WiFi.localIP().toString());
  }
  Serial.println("\nReady!\n");
}

void loop() {
  server.handleClient();
  handleNTP();
  handleTCP();
  
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    
    if (gps.encode(c)) {
      updateGPSTime();
    }
    
    if (tcpClient && tcpClient.connected()) {
      tcpClient.write(c);
    }
  }
  
  if (millis() - lastPrintTime > 10000) {
    printGPSStatus();
    lastPrintTime = millis();
  }
  
  if (millis() - lastNetworkCheck > 30000) {
    checkAndReconnect();
    lastNetworkCheck = millis();
  }
}

void checkAndReconnect() {
  if (!ethConnected && !wifiConnected) {
    Serial.println("No network - attempting WiFi reconnect...");
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
  }
}

void handleTCP() {
  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.available();
    if (tcpClient) {
      Serial.println("TCP client connected");
    }
  }
}

void updateGPSTime() {
  if (gps.time.isValid() && gps.date.isValid()) {
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();
    
    int a = (14 - month) / 12;
    int y = year - a;
    int m = month + 12 * a - 3;
    
    unsigned long jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 + 1721119;
    gpsTimeSeconds = (jdn - 2440588) * 86400UL + hour * 3600UL + minute * 60UL + second;
    timeValid = true;
    
    if (ppsReceived) {
      ppsReceived = false;
    }
  }
}

// get current time in ntp format
unsigned long getNTPTimestamp() {
  if (!timeValid) return 0;
  unsigned long elapsedMicros = micros() - ppsTimeMicros;
  unsigned long elapsedSeconds = elapsedMicros / 1000000;
  unsigned long currentTime = gpsTimeSeconds + elapsedSeconds;
  return currentTime + 2208988800UL;
}

// get fraction of a second
unsigned long getNTPFraction() {
  if (!timeValid) return 0;
  unsigned long elapsedMicros = micros() - ppsTimeMicros;
  unsigned long fractionMicros = elapsedMicros % 1000000;
  return (fractionMicros * 4294967296ULL) / 1000000ULL;
}

// https://labs.apnic.net/index.php/2014/03/10/protocol-basics-the-network-time-protocol/
void handleNTP() {
  int packetSize = ntpUDP.parsePacket();
  if (packetSize) {
    byte requestBuffer[NTP_PACKET_SIZE];
    ntpUDP.read(requestBuffer, NTP_PACKET_SIZE);
    
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    
    packetBuffer[0] = 0b00100100;
    packetBuffer[1] = 1;
    packetBuffer[2] = 6;
    packetBuffer[3] = 0xEC;
    
    unsigned long refTimestamp = getNTPTimestamp();
    unsigned long refFraction = getNTPFraction();
    
    packetBuffer[16] = (refTimestamp >> 24) & 0xFF;
    packetBuffer[17] = (refTimestamp >> 16) & 0xFF;
    packetBuffer[18] = (refTimestamp >> 8) & 0xFF;
    packetBuffer[19] = refTimestamp & 0xFF;
    packetBuffer[20] = (refFraction >> 24) & 0xFF;
    packetBuffer[21] = (refFraction >> 16) & 0xFF;
    packetBuffer[22] = (refFraction >> 8) & 0xFF;
    packetBuffer[23] = refFraction & 0xFF;
    
    packetBuffer[24] = requestBuffer[40];
    packetBuffer[25] = requestBuffer[41];
    packetBuffer[26] = requestBuffer[42];
    packetBuffer[27] = requestBuffer[43];
    packetBuffer[28] = requestBuffer[44];
    packetBuffer[29] = requestBuffer[45];
    packetBuffer[30] = requestBuffer[46];
    packetBuffer[31] = requestBuffer[47];
    
    unsigned long rxTimestamp = getNTPTimestamp();
    unsigned long rxFraction = getNTPFraction();
    
    packetBuffer[32] = (rxTimestamp >> 24) & 0xFF;
    packetBuffer[33] = (rxTimestamp >> 16) & 0xFF;
    packetBuffer[34] = (rxTimestamp >> 8) & 0xFF;
    packetBuffer[35] = rxTimestamp & 0xFF;
    packetBuffer[36] = (rxFraction >> 24) & 0xFF;
    packetBuffer[37] = (rxFraction >> 16) & 0xFF;
    packetBuffer[38] = (rxFraction >> 8) & 0xFF;
    packetBuffer[39] = rxFraction & 0xFF;
    
    unsigned long txTimestamp = getNTPTimestamp();
    unsigned long txFraction = getNTPFraction();
    
    packetBuffer[40] = (txTimestamp >> 24) & 0xFF;
    packetBuffer[41] = (txTimestamp >> 16) & 0xFF;
    packetBuffer[42] = (txTimestamp >> 8) & 0xFF;
    packetBuffer[43] = txTimestamp & 0xFF;
    packetBuffer[44] = (txFraction >> 24) & 0xFF;
    packetBuffer[45] = (txFraction >> 16) & 0xFF;
    packetBuffer[46] = (txFraction >> 8) & 0xFF;
    packetBuffer[47] = txFraction & 0xFF;
    
    ntpUDP.beginPacket(ntpUDP.remoteIP(), ntpUDP.remotePort());
    ntpUDP.write(packetBuffer, NTP_PACKET_SIZE);
    ntpUDP.endPacket();
  }
}

String getIPAddress() {
  if (ethConnected) {
    return ETH.localIP().toString();
  } else if (wifiConnected) {
    return WiFi.localIP().toString();
  } else {
    return "No Network";
  }
}

String getConnectionType() {
  if (ethConnected) {
    return "Ethernet";
  } else if (wifiConnected) {
    return "WiFi";
  } else {
    return "Disconnected";
  }
}

// simple status page
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<style>";
  html += "body{font-family:Arial;margin:40px;background:#f0f0f0}";
  html += ".container{background:white;padding:30px;border-radius:8px;max-width:600px;margin:auto;box-shadow:0 2px 4px rgba(0,0,0,0.1)}";
  html += "h1{color:#333;margin-top:0}";
  html += ".info{background:#e3f2fd;padding:15px;border-radius:4px;margin:15px 0}";
  html += ".info p{margin:8px 0}";
  html += ".good{color:#2e7d32}";
  html += ".bad{color:#c62828}";
  html += ".warning{color:#f57c00}";
  html += "a{display:inline-block;background:#2196F3;color:white;padding:10px 20px;text-decoration:none;border-radius:4px;margin-top:10px}";
  html += "a:hover{background:#1976D2}";
  html += "</style></head><body><div class='container'>";
  html += "<h1>ESP32-P4 GPS NTP Server</h1>";
  html += "<div class='info'>";
  html += "<p><strong>Connection:</strong> <span class='" + String((ethConnected || wifiConnected) ? "good" : "bad") + "'>" + getConnectionType() + "</span></p>";
  html += "<p><strong>IP Address:</strong> " + getIPAddress() + "</p>";
  
  if (ethConnected) {
    html += "<p><strong>MAC (ETH):</strong> " + ETH.macAddress() + "</p>";
  }
  if (wifiConnected) {
    html += "<p><strong>MAC (WiFi):</strong> " + WiFi.macAddress() + "</p>";
  }
  
  html += "<p><strong>NTP Port:</strong> " + String(NTP_PORT) + "</p>";
  html += "<p><strong>TCP NMEA Port:</strong> " + String(TCP_PORT) + "</p>";
  html += "<p><strong>Time Valid:</strong> <span class='" + String(timeValid ? "good" : "bad") + "'>" + String(timeValid ? "Yes" : "No") + "</span></p>";
  html += "<p><strong>PPS Active:</strong> <span class='" + String((micros() - ppsTimeMicros < 2000000) ? "good" : "bad") + "'>" + String((micros() - ppsTimeMicros < 2000000) ? "Yes" : "No") + "</span></p>";
  html += "<p><strong>Satellites:</strong> " + String(gps.satellites.value()) + "</p>";
  
  if (gps.location.isValid()) {
    html += "<p><strong>Latitude:</strong> " + String(gps.location.lat(), 6) + "°</p>";
    html += "<p><strong>Longitude:</strong> " + String(gps.location.lng(), 6) + "°</p>";
  }
  
  if (gps.altitude.isValid()) {
    html += "<p><strong>Altitude:</strong> " + String(gps.altitude.meters(), 1) + " m</p>";
  }
  
  html += "<p><strong>TCP Client:</strong> <span class='" + String((tcpClient && tcpClient.connected()) ? "good" : "bad") + "'>" + String((tcpClient && tcpClient.connected()) ? "Connected" : "Not Connected") + "</span></p>";
  html += "</div>";
  html += "<a href='/status'>JSON Status</a>";
  html += "<p style='color:#666;font-size:12px;margin-top:20px'>Page auto-refreshes every 5 seconds</p>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"connectionType\":\"" + getConnectionType() + "\",";
  json += "\"ethConnected\":" + String(ethConnected ? "true" : "false") + ",";
  json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
  json += "\"ip\":\"" + getIPAddress() + "\",";
  json += "\"ntpPort\":" + String(NTP_PORT) + ",";
  json += "\"tcpPort\":" + String(TCP_PORT) + ",";
  json += "\"timeValid\":" + String(timeValid ? "true" : "false") + ",";
  json += "\"ppsActive\":" + String((micros() - ppsTimeMicros < 2000000) ? "true" : "false") + ",";
  json += "\"satellites\":" + String(gps.satellites.value()) + ",";
  json += "\"lastPPS\":" + String((micros() - ppsTimeMicros) / 1000) + ",";
  json += "\"latitude\":" + String(gps.location.lat(), 6) + ",";
  json += "\"longitude\":" + String(gps.location.lng(), 6) + ",";
  json += "\"altitude\":" + String(gps.altitude.meters(), 1) + ",";
  json += "\"tcpConnected\":" + String((tcpClient && tcpClient.connected()) ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

void printGPSStatus() {
  Serial.println("\n=== GPS Status ===");
  Serial.println("Network: " + getConnectionType() + " (" + getIPAddress() + ")");
  
  if (gps.time.isValid()) {
    Serial.printf("Time: %02d:%02d:%02d UTC\n", 
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.println("Time: INVALID");
  }
  
  if (gps.location.isValid()) {
    Serial.printf("Latitude: %.6f\n", gps.location.lat());
    Serial.printf("Longitude: %.6f\n", gps.location.lng());
  } else {
    Serial.println("Location: INVALID");
  }
  
  if (gps.altitude.isValid()) {
    Serial.printf("Altitude: %.1f m\n", gps.altitude.meters());
  } else {
    Serial.println("Altitude: INVALID");
  }
  
  if (gps.satellites.isValid()) {
    Serial.printf("Satellites: %d\n", gps.satellites.value());
  } else {
    Serial.println("Satellites: INVALID");
  }
  
  bool ppsActive = (micros() - ppsTimeMicros) < 2000000;
  Serial.printf("PPS Status: %s\n", ppsActive ? "ACTIVE" : "INACTIVE");
  Serial.printf("Last PPS: %lu ms ago\n", (micros() - ppsTimeMicros) / 1000);
  
  Serial.println("==================\n");
}