#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "YourSSID";
const char* password = "YourPassword";

WebServer server(80);
String lastMsg = "No message received yet";

// Called when client accesses root page
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='2'></head><body>";
  html += "<h1>Last received:</h1><p>" + lastMsg + "</p></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);        // UART with Raspberry Pi
  WiFi.begin(ssid, password);  // Connect to Wi-Fi

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Start web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();

  // Read from UART
  if (Serial.available()) {
    lastMsg = Serial.readStringUntil('\n');
    lastMsg.trim();
    Serial.println("Received: " + lastMsg);
  }
}
