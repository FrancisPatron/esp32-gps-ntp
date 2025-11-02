#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// WiFi options
String ssid = "";
String password = "";

// Adjust for your esp32 board
HardwareSerial gpsSerial(2);
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
#define PPS_PIN 4
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

void IRAM_ATTR ppsInterrupt() {
  unsigned long now = micros();
  if (now - lastPPSMicros > 500000) {
    ppsTimeMicros = now;
    ppsReceived = true;
    lastPPSMicros = now;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP32-P4 GPS NTP Server Starting ===");

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);
  
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
  
  ntpUDP.begin(NTP_PORT);
  tcpServer.begin();
  
  Serial.println("NTP server started on port " + String(NTP_PORT));
  Serial.println("TCP NMEA server started on port " + String(TCP_PORT));
  Serial.println("PPS input on GPIO " + String(PPS_PIN));
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
  html += "a{display:inline-block;background:#2196F3;color:white;padding:10px 20px;text-decoration:none;border-radius:4px;margin-top:10px}";
  html += "a:hover{background:#1976D2}";
  html += "</style></head><body><div class='container'>";
  html += "<h1>ESP32 GPS NTP Server</h1>";
  html += "<div class='info'>";
  html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
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
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
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