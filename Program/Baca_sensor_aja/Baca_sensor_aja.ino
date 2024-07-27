  
 #include <ESP32Servo.h>
 

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




void setup() {
  Serial.begin(115200);

 
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
