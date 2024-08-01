  
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

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation/1000);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation/1000);
#endif
}

//end sensor do


// Variables

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




void setup() {
  Serial.begin(115200);

 
}

void loop() {
     Temperaturet = (uint8_t)READ_TEMP;
     turbidityKolam = getTurbidity(analogRead(TURBIDITY_PIN_KOLAM));
     turbidityDrum = getTurbidity(analogRead(TURBIDITY_PIN_DRUM));
     turbidity3 = getTurbidity(analogRead(TURBIDITY_PIN_3));
     phKolam = getPH(analogRead(PH_PIN_KOLAM));
     phDrum = getPH(analogRead(PH_PIN_DRUM));
     dissolvedOxygen = getDissolvedOxygen(analogRead(DISSOLVED_OXYGEN_PIN),Temperaturet);
     waterLevel1 = digitalRead(WATER_LEVEL_PIN_1);
     waterLevel2 = digitalRead(WATER_LEVEL_PIN_2);

    Serial.print("Turbidity Kolam: ");
    Serial.print(turbidityKolam);
    Serial.println(" NTU, ");

    Serial.print("Turbidity Drum: ");
    Serial.print(turbidityDrum);
    Serial.println(" NTU, ");

    Serial.print("Turbidity 3: ");
    Serial.print(turbidity3);
    Serial.println(" NTU, ");

    Serial.print("pH Kolam: ");
    Serial.print(phKolam);
    Serial.println(", ");

    Serial.print("pH Drum: ");
    Serial.print(phDrum);
    Serial.println(", ");

    Serial.print("Dissolved Oxygen: ");
    Serial.print(dissolvedOxygen);
    Serial.println(" mg/L, ");

    Serial.print("Water Level 1: ");
    Serial.print(waterLevel1);
    Serial.println(", ");

    Serial.print("Water Level 2: ");
    Serial.print(waterLevel2);
    Serial.println();

    delay(5000);

}
