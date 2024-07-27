  #include <WiFi.h>
  #include <WebServer.h>
  #include <EEPROM.h>
  #include "accesspointPage.h"

// Define pins
#define TURBIDITY_PIN_KOLAM 36
#define TURBIDITY_PIN_DRUM 39
#define TURBIDITY_PIN_3 34
#define PH_PIN_KOLAM 35
#define PH_PIN_DRUM 32
#define DISSOLVED_OXYGEN_PIN 33
#define WATER_LEVEL_PIN_1 25
#define WATER_LEVEL_PIN_2 26
#define LOADCELL_DT 19
#define LOADCELL_SCK 23
#define SET_PIN 4
#define RELAY_PIN_SOLENOID_DRUM2_KOLAM 27
#define RELAY_PIN_SOLENOID_PH_ASAM 14
#define RELAY_PIN_SOLENOID_PH_BASA 12
#define RELAY_PIN_SOLENOID_PEMBUANGAN 13
#define RELAY_PIN_SOLENOID_DRUM1_DRUM2 2
#define RELAY_PIN_POMPA_AIR 15
#define RELAY_PIN_POMPA_AKUARIUM 23
#define SERVO_PIN_LONTAR 18
#define SERVO_PIN_KATUP 5

#define EEPROM_SIZE 64

// Variables
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
const float CAL1_V = 1.6;
const uint8_t CAL1_T = 25;

float turbidityKolam, turbidityDrum, turbidity3, phKolam, phDrum, dissolvedOxygen;
bool waterLevel1, waterLevel2;

uint8_t ultrasonicBuffer[4];
float ultrasonicDistance;

//nama dan password access point
const char *apSSID = "Pakan ikan lele";
const char *apPassword = "12345678";

WebServer server(80);

//define ssid dan password wifi input dari webserver
String ssid = "";
String password = "";

//define mqtt
const char* mqttServer = "broker.hivemq.com";
const uint16_t mqttPort = 1883;
const char* mqttPubTopic = "pakanlele/data";
const char* mqttSubTopic = "pakanlele/espcontrol";

//fungsi pembacaan
float getTurbidity(uint16_t adcVal) {
    float volt = adcVal * 3.3 / 4095.;
    return (((-1120.4 * pow(volt, 2)) + (5742.3 * volt) - 4352.9) / 60.);
}

float getPH(uint16_t adcVal) {
    float volt = adcVal * 3.3 / 4095.;
    return (3.5 * volt);
}

float getDissolvedOxygen(uint16_t adcVal, uint8_t temperatureC = 25) {
    float volt = adcVal * 3.3 / 4095.;
    float vSaturation = CAL1_V + 35 * temperatureC - CAL1_T * 35;
    return ((volt * DO_Table[temperatureC] / vSaturation) / 1000.);
}

float getUltrasonicDistance(uint8_t *buffer) {
    int sum = (buffer[0] + buffer[1] + buffer[2]) & 0x00FF;
    if(sum == buffer[3]) {
        return (((buffer[1] << 8) | buffer[2]) / 10.);
    }
    return 0;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleSubmit() {
  ssid = server.arg("SSID");
  password = server.arg("password");
  
  // Save WiFi credentials to EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.writeString(0, ssid);
  EEPROM.writeString(32, password);
  EEPROM.commit();
  
  server.send(200, "text/html", "Configuration Saved. ESP will restart and try to connect to the specified network.");
  
  // Restart the ESP to apply the new WiFi settings
  delay(1000);
  ESP.restart();
}

void startAccessPoint() {
  // Start access point
  WiFi.softAP(apSSID, apPassword);
  Serial.println("Access Point Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start web server
  server.on("/", handleRoot);
  server.on("/submit", handleSubmit);
  server.begin();
  Serial.println("HTTP server started");
}

void setup() {
  Serial.begin(115200);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Read WiFi credentials from EEPROM
  ssid = EEPROM.readString(0);
  password = EEPROM.readString(32);

  // Try to connect to WiFi with stored credentials
  if (ssid.length() > 0 && password.length() > 0) {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
      Serial.print(".");
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      
      // Close access point
      WiFi.softAPdisconnect(true);
      Serial.println("Access Point Closed");
    } else {
      Serial.println("Failed to connect, starting access point...");
      startAccessPoint();
    }
  } else {
    // Start access point if no credentials found
    startAccessPoint();
  }
}

void loop() {
     turbidityKolam = getTurbidity(analogRead(TURBIDITY_PIN_KOLAM));
     turbidityDrum = getTurbidity(analogRead(TURBIDITY_PIN_DRUM));
     turbidity3 = getTurbidity(analogRead(TURBIDITY_PIN_3));
     phKolam = getPH(analogRead(PH_PIN_KOLAM));
     phDrum = getPH(analogRead(PH_PIN_DRUM));
     dissolvedOxygen = getDissolvedOxygen(analogRead(DISSOLVED_OXYGEN_PIN));
     waterLevel1 = digitalRead(WATER_LEVEL_PIN_1);
     waterLevel2 = digitalRead(WATER_LEVEL_PIN_2);

    Serial.print("Turbidity Kolam: ");
    Serial.print(turbidityKolam);
    Serial.print(" NTU, ");

    Serial.println("Turbidity Drum: ");
    Serial.print(turbidityDrum);
    Serial.print(" NTU, ");

    Serial.println("Turbidity 3: ");
    Serial.print(turbidity3);
    Serial.print(" NTU, ");

    Serial.println("pH Kolam: ");
    Serial.print(phKolam);
    Serial.print(", ");

    Serial.println("pH Drum: ");
    Serial.print(phDrum);
    Serial.print(", ");

    Serial.println("Dissolved Oxygen: ");
    Serial.print(dissolvedOxygen);
    Serial.print(" mg/L, ");

    Serial.print("Water Level 1: ");
    Serial.print(waterLevel1);
    Serial.print(", ");

    Serial.println("Water Level 2: ");
    Serial.print(waterLevel2);
    Serial.println();

}
