#include <WiFi.h>
#include <TinyGPS++.h>
#include <WebServer.h>

// 🛜 Wi-Fi credentials
const char* ssid = "emvundhileh";
const char* password = "telidhu@12";

// 📡 GPS pins (TX → 16, RX → 17)
#define RXD2 16
#define TXD2 17

// 💡 Status LEDs
const int wifiLED = 18;
const int gpsLED = 19;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
WebServer server(80);

double latitude = 0.0;
double longitude = 0.0;

// 🌐 Web page handler
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5'/>"
                "<title>ESP32 GPS Tracker</title></head><body style='font-family:Arial;'>"
                "<h2>📍 ESP32 + NEO-6M Live GPS Tracker</h2>";

  if (gps.location.isValid()) {
    html += "<p><b>Latitude:</b> " + String(latitude, 6) + "<br>";
    html += "<b>Longitude:</b> " + String(longitude, 6) + "</p>";
    html += "<iframe width='100%' height='400' frameborder='0' style='border:0' "
            "src='https://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6) +
            "&z=15&output=embed' allowfullscreen></iframe>";
  } else {
    html += "<p style='color:red;'>❌ Waiting for GPS fix...</p>";
  }

  html += "<p>⏱ Refreshes every 5 seconds</p></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(wifiLED, OUTPUT);
  pinMode(gpsLED, OUTPUT);
  digitalWrite(wifiLED, LOW);
  digitalWrite(gpsLED, LOW);

  Serial.println("📶 Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi Connected!");
  Serial.print("🌐 IP Address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(wifiLED, HIGH);

  server.on("/", handleRoot);
  server.begin();
  Serial.println("📡 Web server started. Open this in your browser:");
  Serial.println("👉 http://" + WiFi.localIP().toString());
}

void loop() {
  // 📡 Read all available GPS data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // ✅ If GPS location is valid
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    digitalWrite(gpsLED, HIGH);
    Serial.printf("📍 Lat: %.6f | Lng: %.6f\n", latitude, longitude);
  } else {
    digitalWrite(gpsLED, LOW);
    Serial.println("❌ Waiting for GPS fix...");
  }

  // 🌐 Handle web requests
  server.handleClient();

  // ⏱ Delay 5 seconds between updates
  delay(5000);
}