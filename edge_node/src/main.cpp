#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ============================================================
// BHOOMI KAVACH - ESP32 #1 EDGE / SENSOR NODE
//
// Sensors actually used:
// MPU6050   -> Tilt + vibration
// Soil      -> Soil moisture
// FC-37     -> Rain
// DS18B20   -> Temperature
// SX1278    -> LoRa
//
// VL53L0X is NOT physically used.
// Displacement is software-defined for prototype testing.
// ============================================================


// ============================================================
// PIN DEFINITIONS
// ============================================================

constexpr uint8_t SOIL_PIN = 34;
constexpr uint8_t RAIN_PIN = 35;
constexpr uint8_t DS18B20_PIN = 4;

constexpr uint8_t GREEN_LED = 25;
constexpr uint8_t YELLOW_LED = 27;
constexpr uint8_t ORANGE_LED = 32;
constexpr uint8_t RED_LED = 33;

constexpr uint8_t BUZZER_PIN = 13;

constexpr uint8_t LORA_CS = 5;
constexpr uint8_t LORA_RST = 14;
constexpr uint8_t LORA_DIO0 = 26;

constexpr long LORA_FREQUENCY = 433E6;

constexpr uint32_t SAMPLE_INTERVAL_MS = 2000;
constexpr uint32_t SERIAL_BAUD = 115200;


// ============================================================
// ALERT THRESHOLDS
// ============================================================

constexpr float WARN_TILT_DEG = 0.60f;
constexpr float CRIT_TILT_DEG = 1.20f;

constexpr float WARN_DISP_MM = 28.0f;
constexpr float CRIT_DISP_MM = 48.0f;

constexpr float WARN_VEL_MM_HR = 1.10f;
constexpr float CRIT_VEL_MM_HR = 2.20f;

constexpr float WARN_VIB_G = 0.06f;
constexpr float CRIT_VIB_G = 0.12f;


// ============================================================
// SIMULATED DISPLACEMENT
//
// Change ONLY this value for demonstration:
//
// 10.0 = NORMAL
// 35.0 = WARNING
// 52.0 = CRITICAL
//
// Current value = 52 mm
// Critical threshold = 48 mm
// ============================================================

constexpr float SIMULATED_DISPLACEMENT_MM = 52.0f;


// ============================================================
// SENSOR OBJECTS
// ============================================================

Adafruit_MPU6050 mpu;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);


// ============================================================
// SENSOR STATUS
// ============================================================

bool mpuOk = false;
bool tempOk = false;


// ============================================================
// VARIABLES
// ============================================================

float lastVelocityDisplacementMm = SIMULATED_DISPLACEMENT_MM;

uint32_t previousSampleMs = 0;
uint32_t sequenceNo = 0;


// ============================================================
// SENSOR FRAME
// ============================================================

struct SensorFrame {
  float tiltX;
  float tiltY;
  float tiltMagnitude;

  float displacement;
  float velocity;

  float vibration;

  float temperature;

  int soilRaw;
  int rainRaw;
};


// ============================================================
// LED CONTROL
// ============================================================

void setLeds(
  bool green,
  bool yellow,
  bool orange,
  bool red
) {
  digitalWrite(GREEN_LED, green ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(ORANGE_LED, orange ? HIGH : LOW);
  digitalWrite(RED_LED, red ? HIGH : LOW);
}


// ============================================================
// LOCAL FAIL-SAFE
// ============================================================

void localFailSafe(const SensorFrame &f) {

  bool critical =
      (f.tiltMagnitude >= CRIT_TILT_DEG) ||
      (f.displacement >= CRIT_DISP_MM) ||
      (f.velocity >= CRIT_VEL_MM_HR) ||
      (f.vibration >= CRIT_VIB_G);

  bool warning =
      (f.tiltMagnitude >= WARN_TILT_DEG) ||
      (f.displacement >= WARN_DISP_MM) ||
      (f.velocity >= WARN_VEL_MM_HR) ||
      (f.vibration >= WARN_VIB_G);


  if (critical) {

    setLeds(false, false, false, true);

    digitalWrite(BUZZER_PIN, HIGH);

  }
  else if (warning) {

    setLeds(false, false, true, false);

    digitalWrite(BUZZER_PIN, LOW);

  }
  else {

    setLeds(true, false, false, false);

    digitalWrite(BUZZER_PIN, LOW);
  }
}


// ============================================================
// MPU6050
//
// Returns vibration.
// Also calculates tilt X and tilt Y.
// ============================================================

float readTiltAndVibration(
  float &tiltX,
  float &tiltY
) {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(
    &accel,
    &gyro,
    &temp
  );


  const float ax = accel.acceleration.x;
  const float ay = accel.acceleration.y;
  const float az = accel.acceleration.z;


  // Tilt X
  tiltX =
      atan2f(
        ax,
        sqrtf(
          ay * ay +
          az * az
        )
      )
      * 180.0f / PI;


  // Tilt Y
  tiltY =
      atan2f(
        ay,
        sqrtf(
          ax * ax +
          az * az
        )
      )
      * 180.0f / PI;


  // Vibration
  const float g = 9.80665f;

  const float accelerationMagnitude =
      sqrtf(
        ax * ax +
        ay * ay +
        az * az
      );


  return fabsf(
    accelerationMagnitude - g
  ) / g;
}


// ============================================================
// DISPLACEMENT
//
// VL53L0X is NOT used.
//
// The value comes directly from:
// SIMULATED_DISPLACEMENT_MM
// ============================================================

float readDisplacementMm() {

  return SIMULATED_DISPLACEMENT_MM;
}


// ============================================================
// TEMPERATURE
// ============================================================

float readTemperature() {

  if (!tempOk) {
    return NAN;
  }


  ds18b20.requestTemperatures();


  float temperature =
      ds18b20.getTempCByIndex(0);


  if (
    temperature ==
    DEVICE_DISCONNECTED_C
  ) {
    return NAN;
  }


  return temperature;
}


// ============================================================
// SEND LoRa PACKET
//
// Packet format:
//
// BK01,
// sequence,
// tilt_x,
// tilt_y,
// tilt_magnitude,
// displacement,
// velocity,
// vibration,
// temperature,
// soil,
// rain
// ============================================================

void sendLoRa(
  const SensorFrame &f
) {

  LoRa.beginPacket();

  LoRa.print("BK01,");

  LoRa.print(sequenceNo);

  LoRa.print(',');
  LoRa.print(f.tiltX, 3);

  LoRa.print(',');
  LoRa.print(f.tiltY, 3);

  LoRa.print(',');
  LoRa.print(f.tiltMagnitude, 3);

  LoRa.print(',');
  LoRa.print(f.displacement, 2);

  LoRa.print(',');
  LoRa.print(f.velocity, 3);

  LoRa.print(',');
  LoRa.print(f.vibration, 4);

  LoRa.print(',');
  LoRa.print(f.temperature, 2);

  LoRa.print(',');
  LoRa.print(f.soilRaw);

  LoRa.print(',');
  LoRa.print(f.rainRaw);

  LoRa.endPacket();
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(SERIAL_BAUD);

  delay(500);


  Serial.println();
  Serial.println("================================");
  Serial.println("BHOOMI KAVACH EDGE NODE");
  Serial.println("ESP32 #1");
  Serial.println("================================");


  // ----------------------------------------------------------
  // LEDS + BUZZER
  // ----------------------------------------------------------

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  setLeds(
    false,
    false,
    false,
    false
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(21, 22);


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  mpuOk =
      mpu.begin(
        0x68,
        &Wire
      );


  if (mpuOk) {

    mpu.setAccelerometerRange(
      MPU6050_RANGE_4_G
    );

    mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
    );

    mpu.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );

    Serial.println(
      "MPU6050_OK"
    );

  }
  else {

    Serial.println(
      "MPU6050_FAIL"
    );
  }


  // ----------------------------------------------------------
  // DS18B20
  // ----------------------------------------------------------

  pinMode(
    DS18B20_PIN,
    INPUT_PULLUP
  );

  ds18b20.begin();

  tempOk =
      ds18b20.getDeviceCount() > 0;


  if (tempOk) {

    Serial.println(
      "DS18B20_OK"
    );

  }
  else {

    Serial.println(
      "DS18B20_NOT_FOUND"
    );
  }


  // ----------------------------------------------------------
  // ANALOG SENSORS
  // ----------------------------------------------------------

  pinMode(
    SOIL_PIN,
    INPUT
  );

  pinMode(
    RAIN_PIN,
    INPUT
  );


  // ----------------------------------------------------------
  // LoRa
  // ----------------------------------------------------------

  SPI.begin(
    18,
    19,
    23,
    5
  );

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );


  if (!LoRa.begin(LORA_FREQUENCY)) {

    Serial.println(
      "LORA_INIT_FAIL"
    );

    setLeds(
      false,
      false,
      false,
      true
    );


    while (true) {

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

      delay(100);

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      delay(900);
    }
  }


  LoRa.setTxPower(17);

  LoRa.setSpreadingFactor(9);

  LoRa.setSignalBandwidth(
    125E3
  );

  LoRa.setCodingRate4(8);

  LoRa.enableCrc();


  Serial.println(
    "LORA_OK_433MHZ"
  );


  // ----------------------------------------------------------
  // DISPLACEMENT INFORMATION
  // ----------------------------------------------------------

  Serial.print(
    "SIMULATED_DISPLACEMENT_MM = "
  );

  Serial.println(
    SIMULATED_DISPLACEMENT_MM
  );

  Serial.println(
    "VL53L0X_NOT_USED"
  );


  // ----------------------------------------------------------
  // START
  // ----------------------------------------------------------

  previousSampleMs =
      millis();


  setLeds(
    true,
    false,
    false,
    false
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  const uint32_t now =
      millis();


  if (
    now - previousSampleMs
    < SAMPLE_INTERVAL_MS
  ) {
    return;
  }


  previousSampleMs =
      now;

  sequenceNo++;


  // ----------------------------------------------------------
  // CREATE SENSOR FRAME
  // ----------------------------------------------------------

  SensorFrame f{};


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  if (mpuOk) {

    f.vibration =
        readTiltAndVibration(
          f.tiltX,
          f.tiltY
        );
  }


  f.tiltMagnitude =
      sqrtf(
        f.tiltX * f.tiltX +
        f.tiltY * f.tiltY
      );


  // ----------------------------------------------------------
  // SIMULATED DISPLACEMENT
  // ----------------------------------------------------------

  f.displacement =
      readDisplacementMm();


  // ----------------------------------------------------------
  // VELOCITY
  // ----------------------------------------------------------

  const float dtHours =
      (
        SAMPLE_INTERVAL_MS
        / 1000.0f
      )
      / 3600.0f;


  if (dtHours > 0.0f) {

    f.velocity =
        fabsf(
          f.displacement -
          lastVelocityDisplacementMm
        )
        / dtHours;

  }
  else {

    f.velocity = 0.0f;
  }


  if (
    !isfinite(f.velocity) ||
    f.velocity > 100.0f
  ) {

    f.velocity = 0.0f;
  }


  lastVelocityDisplacementMm =
      f.displacement;


  // ----------------------------------------------------------
  // TEMPERATURE
  // ----------------------------------------------------------

  f.temperature =
      readTemperature();


  if (
    !isfinite(f.temperature)
  ) {

    f.temperature = 0.0f;
  }


  // ----------------------------------------------------------
  // SOIL MOISTURE
  // ----------------------------------------------------------

  f.soilRaw =
      analogRead(
        SOIL_PIN
      );


  // ----------------------------------------------------------
  // RAIN
  // ----------------------------------------------------------

  f.rainRaw =
      analogRead(
        RAIN_PIN
      );


  // ----------------------------------------------------------
  // LOCAL ALERT
  // ----------------------------------------------------------

  localFailSafe(f);


  // ----------------------------------------------------------
  // SEND LoRa
  // ----------------------------------------------------------

  sendLoRa(f);


  // ----------------------------------------------------------
  // SERIAL MONITOR
  // ----------------------------------------------------------

  Serial.printf(
    "FRAME seq=%lu "
    "tilt=%.3f "
    "disp=%.2f "
    "vel=%.3f "
    "vib=%.4f "
    "temp=%.2f "
    "soil=%d "
    "rain=%d\n",

    (unsigned long)sequenceNo,

    f.tiltMagnitude,

    f.displacement,

    f.velocity,

    f.vibration,

    f.temperature,

    f.soilRaw,

    f.rainRaw
  );
}