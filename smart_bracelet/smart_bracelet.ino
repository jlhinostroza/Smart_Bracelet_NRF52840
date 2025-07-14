#include "LSM6DS3.h"
#include "Wire.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <ArduinoBLE.h>

BLEService serviceBracelet("180A");

BLECharacteristic ppgChar("d44f8caa-fffb-4eb1-b733-7e5511d4ef1c", BLENotify, sizeof(int32_t));
BLECharacteristic hrChar("2A37", BLENotify, sizeof(int16_t));
BLECharacteristic presenceChar("2AE2", BLENotify, sizeof(uint8_t));
BLECharacteristic tempChar("2A6E", BLENotify, sizeof(int16_t));
BLECharacteristic spo2Char("8b6ff090-c1a1-42e9-a4b9-3eefc6530c4b", BLENotify, sizeof(int16_t));
BLECharacteristic accChar("ad0c0caa-3d8c-4b59-a091-b3389da3a0df", BLENotify, 3 * sizeof(float));
BLECharacteristic gyrChar("5da304df-e10c-4270-92da-75487e95094a", BLENotify, 3 * sizeof(float));
BLECharacteristic startChar("f37ac038-0c83-4f93-9bfa-109ec42d1c35", BLENotify, sizeof(uint8_t));

const int buttonPin = D3;
const int ledRojo = LED_RED;
const int ledVerde = LED_GREEN;
const int ledAzul = LED_BLUE;

LSM6DS3 myIMU(I2C_MODE, 0x6A);
MAX30105 particleSensor;

uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

bool samplingActive = false;
unsigned long samplingStartTime = 0;
const unsigned long samplingDuration = 20000;

void setup() {
  Serial.begin(9600);
  delay(2000);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAzul, OUTPUT);
  digitalWrite(ledRojo, HIGH);
  digitalWrite(ledVerde, HIGH);
  digitalWrite(ledAzul, HIGH);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 no detectado.");
    while (1);
  }
  particleSensor.setup(0x1F, 2, 3, 100, 411, 4096);
  particleSensor.enableDIETEMPRDY();

  if (myIMU.begin() != 0) Serial.println("Fallo LSM6DS3");

  if (!BLE.begin()) Serial.println("Fallo BLE");
  BLE.setLocalName("UTP+ Salud - Brazalete");
  BLE.setAdvertisedService(serviceBracelet);

  serviceBracelet.addCharacteristic(ppgChar);
  serviceBracelet.addCharacteristic(hrChar);
  serviceBracelet.addCharacteristic(spo2Char);
  serviceBracelet.addCharacteristic(presenceChar);
  serviceBracelet.addCharacteristic(tempChar);
  serviceBracelet.addCharacteristic(accChar);
  serviceBracelet.addCharacteristic(gyrChar);
  serviceBracelet.addCharacteristic(startChar);

  BLE.addService(serviceBracelet);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Conectado: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      static bool lastButtonState = HIGH;
      bool currentButtonState = digitalRead(buttonPin);

      if (lastButtonState == HIGH && currentButtonState == LOW && !samplingActive) {
        uint8_t one = 1;
        startChar.writeValue(&one, sizeof(one));  // Notificar a la app primero
        delay(100);  // pequeña pausa para sincronización
        countdownAndStartSampling();              // luego iniciar el conteo local
      }
      lastButtonState = currentButtonState;

      if (samplingActive) {
        unsigned long elapsed = millis() - samplingStartTime;

        if (elapsed < samplingDuration) {
          long ir = particleSensor.getIR();
          long red = particleSensor.getRed();
          bool presence = (ir > 50000);

          if (presence) {
            blinkLedWithHeartBeat(elapsed);

            int32_t beat = ir;
            ppgChar.writeValue((uint8_t*)&beat, sizeof(beat));

            uint8_t presenceByte = 1;
            presenceChar.writeValue(&presenceByte, sizeof(presenceByte));

            int16_t temp = (int16_t)(particleSensor.readTemperature() * 100);
            tempChar.writeValue((uint8_t*)&temp, sizeof(temp));

            float acc[3] = { myIMU.readFloatAccelX(), myIMU.readFloatAccelY(), myIMU.readFloatAccelZ() };
            accChar.writeValue((uint8_t*)acc, sizeof(acc));

            float gyr[3] = { myIMU.readFloatGyroX(), myIMU.readFloatGyroY(), myIMU.readFloatGyroZ() };
            gyrChar.writeValue((uint8_t*)gyr, sizeof(gyr));

            static uint8_t sampleCount = 0;
            if (sampleCount < 100) {
              irBuffer[sampleCount] = ir;
              redBuffer[sampleCount] = red;
              sampleCount++;
            }
          } else {
            Serial.println("PR:0 - Sin presencia.");
            samplingActive = false;
            digitalWrite(ledAzul, HIGH);
            digitalWrite(ledRojo, LOW);
            delay(500);
            digitalWrite(ledRojo, HIGH);
          }
        } else {
          processAndPrintHRSpO2();
          samplingActive = false;
          digitalWrite(ledAzul, HIGH);

          int16_t hr = (validHeartRate ? heartRate : 0);
          int16_t ox = (validSPO2 ? spo2 : 0);
          hrChar.writeValue((uint8_t*)&hr, sizeof(hr));
          spo2Char.writeValue((uint8_t*)&ox, sizeof(ox));

          Serial.println("Muestreo finalizado.");
          digitalWrite(ledVerde, LOW);
          delay(400);
          digitalWrite(ledVerde, HIGH);
          delay(100);
          digitalWrite(ledVerde, LOW);
          delay(400);
          digitalWrite(ledVerde, HIGH);
        }
      }
    }

    Serial.println("Desconectado.");
    uint8_t zero = 0;
    startChar.writeValue(&zero, sizeof(zero));
    BLE.advertise();
  }
  delay(50);
}

void countdownAndStartSampling() {
  for (int i = 3; i > 0; i--) {
    Serial.print(i); Serial.println("...");
    digitalWrite(ledVerde, LOW); delay(100);
    digitalWrite(ledVerde, HIGH); delay(900);
  }

  for (int i = 0; i < 100; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }

  spo2 = 0;
  heartRate = 0;
  validSPO2 = 0;
  validHeartRate = 0;
  samplingActive = true;
  samplingStartTime = millis();
}

void blinkLedWithHeartBeat(unsigned long elapsed) {
  int bpm = (validHeartRate > 30 && validHeartRate < 180) ? validHeartRate : 75;
  unsigned long beatPeriod = 60000UL / bpm;
  unsigned long timeInCycle = elapsed % beatPeriod;
  digitalWrite(ledAzul, timeInCycle < 200 ? LOW : HIGH);
}

void processAndPrintHRSpO2() {
  for (byte i = 0; i < 100; i++) {
    while (!particleSensor.available()) {
      particleSensor.check(); delay(5);
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  if (!validHeartRate) heartRate = 0;
  if (!validSPO2) spo2 = 0;
  Serial.print("HR calculado: ");
  Serial.println(heartRate);
  Serial.print("SpO2 calculado: ");
  Serial.println(spo2);
}
