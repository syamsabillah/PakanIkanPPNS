#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

// port webserver 80
AsyncWebServer server(80);

// ssid dan password WiFi
String ssid, password;

// Pin LED
String ledState;
const int ledPin = 2;

// Mengambil data dari Web
String file_awal;
const char* p_ssid = "ssid";
const char* p_password = "password";
String tmp_ssid, tmp_password;

// fungsi membaca file dari SPIFFS
String baca_file(String path){
  String isi_file;
  File file = SPIFFS.open(path, "r");
  if(!file){
    return isi_file;
  }
  while(file.available()){
    isi_file = file.readStringUntil('\n');
    isi_file.trim(); 
  }
  file.close();
  return isi_file;
}

// Menyimpan data ke SPIFFS
void tulis_file(String path, String isi){
  File file = SPIFFS.open(path, "w");
  file.println(isi);
  file.close();
}

// callback dalam web
String processor(const String& var) {
  if(var == "STATE") {
    if(digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    return ledState;
  }
  return String();
}
void setup() {
  // baurate serial dan init pin LED
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // init SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("Gagal Init SPIFFS");
  }
  
  // baca file ssid, pass
  ssid = baca_file("/ssid.txt");
  password = baca_file("/password.txt");
  
  if(ssid == ""){
    Serial.println("SSID Kosong, SSID Default: WiFi Manager");
    ssid = "WiFi Manager";
  }
  
  if(password == ""){
    WiFi.softAP(ssid);  
  }
  else{
    WiFi.softAP(ssid, password);
  }
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("IP address: ");
  Serial.println(myIP);

  if(digitalRead(ledPin)){
    file_awal = "/on.html";
  }
  else{
    file_awal = "/off.html";
  }
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS,  file_awal, "text/html", false, processor);
  });
  server.serveStatic("/", SPIFFS, "/");
  
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin, HIGH);
    Serial.println("LED ON");
    request->send(SPIFFS, "/on.html", "text/html", false, processor);
  });

  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin, LOW);
    Serial.println("LED OFF");
    request->send(SPIFFS, "/off.html", "text/html", false, processor);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/config.html", "text/html");
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam(p_ssid)) {
      tmp_ssid = request->getParam(p_ssid)->value();
      tulis_file("/ssid.txt", tmp_ssid);
      Serial.print("SSID: ");
      Serial.println(tmp_ssid);
    }
    if (request->hasParam(p_password)) {
      tmp_password = request->getParam(p_password)->value();
      tulis_file("/password.txt", tmp_password);
      Serial.print("Pass: ");
      Serial.println(tmp_password);
    }
    
    if(digitalRead(ledPin)){
      file_awal = "/on.html";
    }
    else{
      file_awal = "/off.html";
    }
    request->send(SPIFFS, file_awal, "text/html", false, processor);
    Serial.println("Sebentar Lagi ESP akan di Restart");
    delay(3000);
    ESP.restart();
  });

  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
