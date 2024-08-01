#pragma region Include Libraries
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino_KNN.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include <HX711.h>
#pragma endregion

#pragma region Pin Definitions
#define TURBIDITY_PIN_KOLAM 3
#define TURBIDITY_PIN_DRUM 4
#define TURBIDITY_PIN_3 5
#define PH_PIN_KOLAM 6
#define PH_PIN_DRUM 7
#define DISSOLVED_OXYGEN_PIN 8
#define WATER_LEVEL_PIN_1 24
#define WATER_LEVEL_PIN_2 23
#define LOADCELL_DT 37
#define LOADCELL_SCK 31

#define SET_PIN 4

#define RELAY_PIN_SOLENOID_DRUM2_KOLAM 27
#define RELAY_PIN_SOLENOID_PH_ASAM 14
#define RELAY_PIN_SOLENOID_PH_BASA 12
#define RELAY_PIN_SOLENOID_PEMBUANGAN 13
#define RELAY_PIN_SOLENOID_DRUM1_DRUM2 26
#define RELAY_PIN_POMPA_AIR 25
#define RELAY_PIN_POMPA_AKUARIUM 20
#define SERVO_PIN_LONTAR 18
#define SERVO_PIN_KATUP 5
#pragma endregion

#pragma region HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Login</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f4f4;
        }

        .container {
            width: 300px;
            margin: 0 auto;
            padding: 20px;
            background-color: #fff;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            margin-top: 100px;
        }

        h2 {
            text-align: center;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            font-weight: bold;
        }

        input[type="text"],
        input[type="password"] {
            width: 90%;
            padding: 10px;
            border: 1px solid #ccc;
            border-radius: 5px;
        }

        input[type="submit"] {
            background-color: #007BFF;
            color: #fff;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }

        input[type="submit"]:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>WiFi Login</h2>
        <form action="/" method="post">
            <div class="form-group">
                <label for="SSID">SSID:</label>
                <input type="text" id="SSID" name="SSID" required>
            </div>
            <div class="form-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password">
                <input type="checkbox" id="showPassword"> Show Password<br>
            </div>
            <div class="form-group">
                <input type="submit" value="Save and Restart">
            </div>
        </form>
        <script>
        const passwordInput = document.getElementById('password');
        const showPasswordCheckbox = document.getElementById('showPassword');

        showPasswordCheckbox.addEventListener('change', function () {
            if (showPasswordCheckbox.checked) {
                passwordInput.type = 'text';
            } else {
                passwordInput.type = 'password';
            }
        });
    </script>
    </div>
</body>
</html>
)rawliteral";
#pragma endregion

//Sensor DO

#define VREF 3300    //VREF (mv)
#define ADC_RES 4095 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (22) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (22)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;


//end sensor do

#pragma region Variables

float turbidityKolam, turbidityDrum, turbidity3, phKolam, phDrum, dissolvedOxygen;
bool waterLevel1, waterLevel2;

uint8_t ultrasonicBuffer[4];
float ultrasonicDistance;

String STA_SSID;
String STA_PASS;
String AP_SSID = "Pakan Lele - IoT";
String AP_PASS = "12345678";

const char* mqttServer = "broker.hivemq.com";
const uint16_t mqttPort = 1883;
const char* mqttPubTopic = "pakanlele/data";
const char* mqttSubTopic = "pakanlele/espcontrol";

const char* ntpServer = "pool.ntp.org";
int8_t gmtOffset = 7;
bool dstOffset = false;
#define TOTAL_TIMEOFFSET (gmtOffset * 3600) + (dstOffset * 3600)
char dateTimeBuffer[20], gmtDstBuffer[10];
struct tm timeinfo;

int knnClassification_Turbidity, knnClassification_PH;
float knnConfidence_Turbidity, knnConfidence_PH;
typedef enum {
    KNN_TURBIDITY_KERUH,
    KNN_TURBIDITY_JERNIH
} KNN_TURBIDITY;
typedef enum {
    KNN_PH_NORMAL,
    KNN_PH_ASAM,
    KNN_PH_BASA
} KNN_PH;

typedef enum {
    CIRCULATION_SEDOT_AIR,
    CIRCULATION_PENGENDAPAN,
    CIRCULATION_BUANG_ENDAPAN,
    CIRCULATION_PINDAH_DRUM,
    CIRCULATION_CEK_PH,
    CIRCULATION_KEMBALIKAN_AIR
} CIRCULATION;
CIRCULATION circulationState = CIRCULATION_SEDOT_AIR;
bool circulationInProgress, leleSudahMakan;
unsigned long finishPengendapan;

#define LOADCELL_CAL -471.497
float loadCellVal;
#pragma endregion

#pragma region Objects
Preferences pref;
AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);
KNNClassifier knnTurbidity(1);
KNNClassifier knnPH(1);
Servo servoLontar;
Servo servoKatup;
ESP32Time rtc(TOTAL_TIMEOFFSET);
HX711 loadCell;

TaskHandle_t SensorReading_Handle;
TaskHandle_t SendMQTT_Handle;
TaskHandle_t CirculationTask_Handle;
TaskHandle_t ServoTask_Handle;
#pragma endregion

#pragma region Functions
float getTurbidity(uint16_t adcVal) {
    float volt = adcVal * 3.3 / 4095.;
    return (((-1120.4 * pow(volt, 2)) + (5742.3 * volt) - 4352.9) / 60.);
}

float getPH(uint16_t adcVal) {
    float volt = adcVal * 3.3 / 4095.;
    return (3.5 * volt);
}

float getDissolvedOxygen(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation/1000);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation/1000);
#endif
}

float getUltrasonicDistance(uint8_t *buffer) {
    int sum = (buffer[0] + buffer[1] + buffer[2]) & 0x00FF;
    if(sum == buffer[3]) {
        return (((buffer[1] << 8) | buffer[2]) / 10.);
    }
    return 0;
}

void startAP() {
    log_n("Starting AP with name %s", AP_SSID.c_str());
    log_n("Password: %s", AP_PASS.c_str());
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
        int params = request->params();
        for(int i = 0; i < params; i++) {
            AsyncWebParameter* p = request->getParam(i);
            if(p->isPost()) {
                if(p->name() == "SSID") {
                    pref.putString("STA_SSID", p->value().c_str());
                    log_n("SSID set to %s", pref.getString("STA_SSID", "NoSSID").c_str());
                }
                if(p->name() == "password") {
                    pref.putString("STA_PASS", p->value().c_str());
                    log_n("Password set to %s", pref.getString("STA_PASS", "NoPass").c_str());
                }
            }
        }
        request->send(200, "text/html", index_html);
        delay(2000);
        ESP.restart();
    });

    server.begin();
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    log_n("Connection to %s lost", STA_SSID.c_str());
    WiFi.reconnect();
}

void messageHandler(char *topic, uint8_t *payload, unsigned int lenght) {
    log_n("Incoming message from %s", topic);

    JsonDocument doc;
    deserializeJson(doc, payload);
    log_n("Payload: %s", doc.as<String>().c_str());
    
    digitalWrite(RELAY_PIN_SOLENOID_PEMBUANGAN, doc["Solenoid Buang"].as<bool>());
    digitalWrite(RELAY_PIN_SOLENOID_DRUM1_DRUM2, doc["Solenoid Pindah"].as<bool>());
    digitalWrite(RELAY_PIN_SOLENOID_DRUM2_KOLAM, doc["Solenoid Kembali"].as<bool>());
    digitalWrite(RELAY_PIN_POMPA_AIR, doc["Pompa"].as<bool>());

    if(doc["Servo"] == "Beri Makan") {
        if(eTaskGetState(ServoTask_Handle) != eRunning) {
            log_n("Memberi makan!");
            vTaskResume(ServoTask_Handle);
        }
    }
}
#pragma endregion

#pragma region Task Codes
void SensorReading(void *pvParameters) {
    for(;;) {
      Temperaturet = (uint8_t)READ_TEMP;
        turbidityKolam = getTurbidity(analogRead(TURBIDITY_PIN_KOLAM));
        turbidityDrum = getTurbidity(analogRead(TURBIDITY_PIN_DRUM));
        turbidity3 = getTurbidity(analogRead(TURBIDITY_PIN_3));
        phKolam = getPH(analogRead(PH_PIN_KOLAM));
        phDrum = getPH(analogRead(PH_PIN_DRUM));
        dissolvedOxygen = getDissolvedOxygen(analogRead(DISSOLVED_OXYGEN_PIN),Temperaturet);
        waterLevel1 = digitalRead(WATER_LEVEL_PIN_1);
        waterLevel2 = digitalRead(WATER_LEVEL_PIN_2);

        knnClassification_Turbidity = knnTurbidity.classify(&turbidityKolam, 3);
        knnConfidence_Turbidity = knnTurbidity.confidence();
        
        knnClassification_PH = knnPH.classify(&phKolam, 1);
        knnConfidence_PH = knnPH.confidence();

        log_n("\nTurbidity Kolam: %.2f\nTurbidity Drum: %.2f\nTurbidity 3: %.2f\npH Kolam: %.2f\npH Drum: %.2f\nDissolved Oxygen: %.2f\nWater Level 1: %d\nWater Level 2: %d\nUltrasonic Distance: %.2f\nLoad Cell: %.2f", turbidityKolam, turbidityDrum, turbidity3, phKolam, phDrum, dissolvedOxygen, waterLevel1, waterLevel2, ultrasonicDistance, loadCellVal);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void SendMQTT(void *pvParameters) {
    for(;;) {
        JsonDocument doc;
        doc["Turbidity Kolam"] = turbidityKolam;
        doc["pH Kolam"] = phKolam;
        doc["Dissolved Oxygen"] = dissolvedOxygen;
        doc["WL Ultrasonik"] = ultrasonicDistance;
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);

        mqtt.publish(mqttPubTopic, jsonBuffer);
        log_n("MQTT: %s", jsonBuffer);
        
        vTaskDelay(10 * 60 * 1000 / portTICK_PERIOD_MS);
    }
}

void CirculationTask(void *pvParameters) {
    for(;;) {
        switch(circulationState) {
            case CIRCULATION_SEDOT_AIR:
                if(ultrasonicDistance <= 15) {
                    digitalWrite(RELAY_PIN_POMPA_AIR, LOW);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    finishPengendapan = rtc.getEpoch() + (60 * 60 * 24);
                    log_n("Waktunya pengendapan!");
                    circulationState = CIRCULATION_PENGENDAPAN;
                }
                break;

            case CIRCULATION_PENGENDAPAN:
                if(knnClassification_Turbidity == KNN_TURBIDITY_KERUH) {
                    if(rtc.getEpoch() >= finishPengendapan) {
                        digitalWrite(RELAY_PIN_SOLENOID_PEMBUANGAN, HIGH);
                        finishPengendapan += 60;
                        log_n("Waktunya buang endapan!");
                        circulationState = CIRCULATION_BUANG_ENDAPAN;
                    }
                } else {
                    digitalWrite(RELAY_PIN_SOLENOID_DRUM1_DRUM2, HIGH);
                    log_n("Waktunya pindah drum!");
                    circulationState = CIRCULATION_PINDAH_DRUM;
                }
                break;

            case CIRCULATION_BUANG_ENDAPAN:
                if(rtc.getEpoch() >= finishPengendapan || knnClassification_Turbidity == KNN_TURBIDITY_JERNIH) {
                    digitalWrite(RELAY_PIN_SOLENOID_PEMBUANGAN, LOW);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_PIN_SOLENOID_DRUM1_DRUM2, HIGH);
                    log_n("Waktunya pindah drum!");
                    circulationState = CIRCULATION_PINDAH_DRUM;
                }
                break;

            case CIRCULATION_PINDAH_DRUM:
                if(ultrasonicDistance >= 90) {
                    digitalWrite(RELAY_PIN_SOLENOID_DRUM1_DRUM2, LOW);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    log_n("Waktunya cek pH!");
                    circulationState = CIRCULATION_CEK_PH;
                }
                break;

            case CIRCULATION_CEK_PH:
                if(knnClassification_PH == KNN_PH_ASAM) {
                    digitalWrite(RELAY_PIN_SOLENOID_PH_ASAM, HIGH);
                    digitalWrite(RELAY_PIN_SOLENOID_PH_BASA, LOW);
                } else if(knnClassification_PH == KNN_PH_BASA) {
                    digitalWrite(RELAY_PIN_SOLENOID_PH_ASAM, LOW);
                    digitalWrite(RELAY_PIN_SOLENOID_PH_BASA, HIGH);
                } else {
                    digitalWrite(RELAY_PIN_SOLENOID_PH_ASAM, LOW);
                    digitalWrite(RELAY_PIN_SOLENOID_PH_BASA, LOW);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_PIN_SOLENOID_DRUM2_KOLAM, HIGH);
                    log_n("Waktunya kembalikan air!");
                    circulationState = CIRCULATION_KEMBALIKAN_AIR;
                }
                break;

            case CIRCULATION_KEMBALIKAN_AIR:
                vTaskDelay(20 * 60 * 1000 / portTICK_PERIOD_MS);
                digitalWrite(RELAY_PIN_SOLENOID_DRUM2_KOLAM, LOW);
                circulationState = CIRCULATION_SEDOT_AIR;
                log_n("Sirkulasi air selesai!");
                vTaskSuspend(CirculationTask_Handle);
                break;

            default:
                break;
        }
    }
}

void ServoTask(void *pvParameters) {
    for(;;) {
        if(loadCellVal >= 10.) {
            servoKatup.write(0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            servoLontar.write(165);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            servoLontar.write(75);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            loadCell.tare();
            log_n("Memberi makan, selesai!");
            vTaskSuspend(ServoTask_Handle);
        } else {
            servoKatup.write(90);
        }
    }
}
#pragma endregion

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);

    #pragma region Pin Mode
    pinMode(TURBIDITY_PIN_KOLAM, INPUT);
    pinMode(TURBIDITY_PIN_DRUM, INPUT);
    pinMode(TURBIDITY_PIN_3, INPUT);
    pinMode(PH_PIN_KOLAM, INPUT);
    pinMode(PH_PIN_DRUM, INPUT);
    pinMode(DISSOLVED_OXYGEN_PIN, INPUT);
    pinMode(WATER_LEVEL_PIN_1, INPUT);
    pinMode(WATER_LEVEL_PIN_2, INPUT);
    pinMode(SET_PIN, INPUT_PULLUP);
    pinMode(RELAY_PIN_SOLENOID_DRUM2_KOLAM, OUTPUT);
    pinMode(RELAY_PIN_SOLENOID_PH_ASAM, OUTPUT);
    pinMode(RELAY_PIN_SOLENOID_PH_BASA, OUTPUT);
    pinMode(RELAY_PIN_SOLENOID_PEMBUANGAN, OUTPUT);
    pinMode(RELAY_PIN_SOLENOID_DRUM1_DRUM2, OUTPUT);
    pinMode(RELAY_PIN_POMPA_AIR, OUTPUT);
    pinMode(RELAY_PIN_POMPA_AKUARIUM, OUTPUT);

    digitalWrite(RELAY_PIN_SOLENOID_DRUM2_KOLAM, LOW);
    digitalWrite(RELAY_PIN_SOLENOID_PH_ASAM, LOW);
    digitalWrite(RELAY_PIN_SOLENOID_PH_BASA, LOW);
    digitalWrite(RELAY_PIN_SOLENOID_PEMBUANGAN, LOW);
    digitalWrite(RELAY_PIN_SOLENOID_DRUM1_DRUM2, LOW);
    digitalWrite(RELAY_PIN_POMPA_AIR, LOW);
    digitalWrite(RELAY_PIN_POMPA_AKUARIUM, LOW);
    #pragma endregion

    #pragma region Servo Initialization
    servoKatup.setPeriodHertz(50);
    servoLontar.setPeriodHertz(50);

    servoKatup.attach(SERVO_PIN_KATUP);
    servoLontar.attach(SERVO_PIN_LONTAR);

    servoKatup.write(0);
    servoLontar.write(75);
    #pragma endregion

    pref.begin("Credentials");
    
    STA_SSID = pref.getString("STA_SSID", "NoSSID");
    STA_PASS = pref.getString("STA_PASS", "NoPASS");

//    if(!digitalRead(SET_PIN)) {
//        startAP();
//    } else {
        log_n("Connecting to %s", STA_SSID.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.begin(STA_SSID, STA_PASS);
        delay(5000);
        if(WiFi.status() != WL_CONNECTED) {
            log_n("Can't connect to %s", STA_SSID.c_str());
            startAP();
        } else {
            log_n("Connected to %s", STA_SSID.c_str());
            WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
            mqtt.setServer(mqttServer, mqttPort);
            mqtt.setCallback(messageHandler);
            while(!mqtt.connect("Pakan Lele")) {
                log_n("Failed to connect to MQTT Broker");
                delay(1000);
            }
            log_n("MQTT connected");
            if(mqtt.subscribe(mqttSubTopic)) {
                log_n("MQTT subscribed");
            } else {
                log_n("MQTT subscription failed");
            }
        }
//    }

    loadCell.begin(LOADCELL_DT, LOADCELL_SCK);
    loadCell.set_scale(LOADCELL_CAL);
    loadCell.tare();

    #pragma region KNN Initialization
    float turbiditySample[4] = {12, 14, 16, 18};
    float pHSample[4] = {5, 6, 7, 8};

    knnTurbidity.addExample(&turbiditySample[0], KNN_TURBIDITY_JERNIH);
    knnTurbidity.addExample(&turbiditySample[1], KNN_TURBIDITY_JERNIH);
    knnTurbidity.addExample(&turbiditySample[2], KNN_TURBIDITY_KERUH);
    knnTurbidity.addExample(&turbiditySample[3], KNN_TURBIDITY_KERUH);
    
    knnPH.addExample(&pHSample[0], KNN_PH_ASAM);
    knnPH.addExample(&pHSample[1], KNN_PH_NORMAL);
    knnPH.addExample(&pHSample[2], KNN_PH_NORMAL);
    knnPH.addExample(&pHSample[3], KNN_PH_BASA);
    #pragma endregion
    
    #pragma region Task Creation
    xTaskCreatePinnedToCore(SensorReading, "Sensor Reading Task", 2048, NULL, 1, &SensorReading_Handle, 1);
    xTaskCreatePinnedToCore(SendMQTT, "Send MQTT Task", 4096, NULL, 1, &SendMQTT_Handle, 1);
    xTaskCreatePinnedToCore(CirculationTask, "Circulation Task", 2048, NULL, 1, &CirculationTask_Handle, 1);
    vTaskSuspend(CirculationTask_Handle);
    xTaskCreatePinnedToCore(ServoTask, "Servo Task", 1024, NULL, 1, &ServoTask_Handle, 1);
    vTaskSuspend(ServoTask_Handle);
    #pragma endregion
}

void loop() {
    if(Serial2.available()) {
        Serial2.readBytes(ultrasonicBuffer, 4);
        if(ultrasonicBuffer[0] == 0xFF) {
            ultrasonicDistance = getUltrasonicDistance(ultrasonicBuffer);
        }
    }

    if(loadCell.is_ready()) {
        loadCellVal = loadCell.get_units(10);
    }

    uint8_t currentHour = rtc.getHour(true);
    if((currentHour == 6 || currentHour == 12 || currentHour == 18) && !leleSudahMakan) {
        if(eTaskGetState(ServoTask_Handle) == eSuspended) {
            log_n("Waktunya makan!");
            vTaskResume(ServoTask_Handle);
        }
        leleSudahMakan = true;
    } else if(currentHour != 6 && currentHour != 12 && currentHour != 18 && leleSudahMakan) {
        leleSudahMakan = false;
    }

    if(turbidityKolam > 15. || phKolam < 5.5 || phKolam > 7.5 || dissolvedOxygen < 7.) {
        if(eTaskGetState(CirculationTask_Handle) == eSuspended) {
            log_n("Waktunya sirkulasi air!");
            vTaskResume(CirculationTask_Handle);
        }
    }
    
    mqtt.loop();
}
