/*
將UART收到的訊息發到自訂網頁檢視訊息
ESP32 UART2 使用 GPIO16(RX), GPIO17(TX)
*/

//IP fixed as 192.168.16.184

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Wifi662-8_2B";
const char* password = "0980566079";
IPAddress local_IP(192, 168, 16, 184);   // 固定 IP，選一個沒被其他設備用的
IPAddress gateway(192, 168, 16, 1);      // 通常是路由器 IP
IPAddress subnet(255, 255, 255, 0);      // 子網掩碼

WebServer server(80);
String lastMsg = "No message received yet";
unsigned long lastMsgTime = 0;

// Called when client accesses root page
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='2'></head><body>";
  html += "<h1>ESP32 UART Monitor</h1>";
  html += "<p><strong>Last received:</strong> " + lastMsg + "</p>";
  html += "<p><strong>Time:</strong> " + String(millis()) + " ms</p>";
  html += "<p><strong>UART Config:</strong> GPIO16(RX), GPIO17(TX) @ 115200 baud</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);  // Debug serial
  delay(1000);  // 等待串口初始化
  Serial.println("ESP32 starting...");
  
  // 初始化 UART2，使用 GPIO16(RX), GPIO17(TX)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // baud, config, RX pin, TX pin
  Serial.println("UART2 initialized on GPIO16(RX), GPIO17(TX)");

  // 設定靜態 IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

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
  Serial.println("Ready to receive UART messages on GPIO16/17");
}

void loop() {
  server.handleClient();

  // Read from UART2 (GPIO16/17)
  if (Serial2.available()) {
    lastMsg = Serial2.readStringUntil('\n');
    lastMsg.trim();
    lastMsgTime = millis();
    
    Serial.println("Received via UART2: " + lastMsg);
    
    // 可選：透過 UART2 發送回應確認
    Serial2.println("ACK: " + lastMsg);
  }
  
  // 清除過時的訊息 (30秒後顯示 "No recent message")
  if (millis() - lastMsgTime > 30000 && lastMsg != "No message received yet") {
    lastMsg = "No recent message (timeout)";
  }
}