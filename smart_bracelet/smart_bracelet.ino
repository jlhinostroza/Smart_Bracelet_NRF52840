#include "LSM6DS3.h"
#include "Wire.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <ArduinoBLE.h>

BLEService serviceBracelet("180A"/*"1840"*/);  //UUID para el servicio de "Generic Health Sensor Service"

// PPG - personalizado
BLECharacteristic ppgChar("d44f8caa-fffb-4eb1-b733-7e5511d4ef1c", BLENotify, 4);  // 4 bytes o según necesites

// HR - estándar BLE
BLECharacteristic hrChar("2A37", BLENotify, 2);  // 2 bytes: frecuencia cardíaca

// Presencia - personalizado
BLECharacteristic presenceChar("2AE2", BLENotify, 1);  // 1 byte booleano

// Temperatura - estándar
BLECharacteristic tempChar("2A6E", BLENotify, 2);  // 2 bytes float o int16_t si usas 0.01°C

// SPO2 - personalizado
BLECharacteristic spo2Char("8b6ff090-c1a1-42e9-a4b9-3eefc6530c4b", BLENotify, 2);

// Acelerómetro (X, Y, Z) - personalizado
BLECharacteristic accChar("ad0c0caa-3d8c-4b59-a091-b3389da3a0df", BLENotify, 4);

// Giroscopio (X, Y, Z) - personalizado
BLECharacteristic gyrChar("5da304df-e10c-4270-92da-75487e95094a", BLENotify, 4);

// Señal de inicio
BLECharacteristic startChar("f37ac038-0c83-4f93-9bfa-109ec42d1c35", BLENotify, 1); // 1 byte: 1 cuando se pulsa


// Pines
const int buttonPin = D3;
const int ledRojo = LED_RED;
const int ledVerde = LED_GREEN;
const int ledAzul = LED_BLUE;

String data1;
String data2;

// Objetos sensores
LSM6DS3 myIMU(I2C_MODE, 0x6A);
MAX30105 particleSensor;

// Buffers para HR y SpO2
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

// Variables control estado
bool samplingActive = false;
unsigned long samplingStartTime = 0;
const unsigned long samplingDuration = 20000;  // 20 segundos

void setup() {
  Serial.begin(9600);
  delay(2000);  // Esperar a que Serial esté listo

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(ledRojo, OUTPUT);
  digitalWrite(ledRojo, HIGH);

  pinMode(ledVerde, OUTPUT);
  digitalWrite(ledVerde, HIGH);

  pinMode(ledAzul, OUTPUT);
  digitalWrite(ledAzul, HIGH);

  Serial.println("Inicializando sensores...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 no detectado. Verifica conexión.");
    while (1);
  }
  Serial.println("MAX30105 iniciado correctamente.");


  particleSensor.setup(0x1F, 2, 3, 100, 411, 4096);  // configuración avanzada
  particleSensor.enableDIETEMPRDY();

  if (myIMU.begin() != 0) {
    Serial.println("Fallo al inicializar LSM6DS3.");
  }
  Serial.println("LSM6DS3 iniciado correctamente.");

  if (!BLE.begin()) {
    Serial.println("Fallo al iniciar BLE");
  }
  Serial.println("BLE iniciado correctamente.");

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
  BLEDevice central = BLE.central();  // crea un objeto central para recibir datos

  if (central) {                            // si se conecta un central
    Serial.print("Conectado a central: ");  // muestra texto por monitor serie
    Serial.println(central.address());      // muestra direccion del central conectado
    digitalWrite(LED_BUILTIN, HIGH);        // enciende LED 

    while (central.connected()) {
      static bool lastButtonState = HIGH;
      bool currentButtonState = digitalRead(buttonPin);

      if (lastButtonState == HIGH && currentButtonState == LOW && !samplingActive) {
        // Botón presionado y no estamos muestreando: iniciar cuenta regresiva
        countdownAndStartSampling();

        // Señal de inicio: enviar 1
        startChar.writeValue((byte)1);
      }
      lastButtonState = currentButtonState;

      if (samplingActive) {
        unsigned long elapsed = millis() - samplingStartTime;

        if (elapsed < samplingDuration) {
          // Leer IR para presencia
          long ir = particleSensor.getIR();
          long red = particleSensor.getRed();
          bool presence = (ir > 50000);

          if (presence) {
            // Parpadear LED imitando latido
            blinkLedWithHeartBeat(elapsed);

            ppgChar.writeValue(String(getHeartBeat(ir)).c_str());
            presenceChar.writeValue(String(getPresence(ir)).c_str());
            tempChar.writeValue(String(getTemperature()).c_str());
            accChar.writeValue(getAcceleration().c_str());
            gyrChar.writeValue(getGyroscope().c_str());

            Serial.print(getHeartBeat(ir) + " ");
            Serial.print(getPresence(ir) + " ");
            Serial.print(String(getTemperature()) + " ");
            Serial.print(getAcceleration() + " ");
            Serial.print(getGyroscope() + " ");

            // Guardar muestras para HR y SpO2
            static uint8_t sampleCount = 0;
            if (sampleCount < 100) {
              irBuffer[sampleCount] = ir;
              redBuffer[sampleCount] = red;
              sampleCount++;
            }
          } else {
            Serial.println("PR:0 - Presencia perdida, interrumpiendo muestreo.");
            samplingActive = false;
            digitalWrite(ledAzul, HIGH);
            digitalWrite(ledRojo, LOW);
            delay(500);
            digitalWrite(ledRojo, HIGH);
          }
        } else {
          // Tiempo cumplido, procesar HR y SpO2
          processAndPrintHRSpO2();
          samplingActive = false;
          digitalWrite(ledAzul, HIGH);

          hrChar.writeValue((int32_t)heartRate);
          spo2Char.writeValue((int32_t)spo2);

          Serial.print(heartRate + " ");
          Serial.print(spo2 + " ");

          Serial.println("Muestreo finalizado. Esperando próximo botón...");
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
    Serial.print("Desconectado de central: ");  // muestra texto al desconectarse el central
    Serial.println(central.address());
    startChar.writeValue((byte)0);  // Reinicio señal
    BLE.advertise(); // <-- REANUDA PUBLICIDAD
  }
  delay(50);
}
// ================= FUNCIONES =================

void countdownAndStartSampling() {
  Serial.println("Preparándose para muestrear...");
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    digitalWrite(ledVerde, LOW);
    Serial.println("...");
    delay(100);
    digitalWrite(ledVerde, HIGH);
    delay(900);
  }
  Serial.println("¡Comenzando muestreo!");

  // Reset buffers y variables
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
  // Simula latido: LED ON ~200ms cada latido, latidos ~60-100 bpm (1 latido ~600-1000ms)
  // Aquí usamos HR estimado si válido, sino latido fijo a 75 bpm (800ms ciclo)
  int bpm = (validHeartRate > 30 && validHeartRate < 180) ? validHeartRate : 75;
  unsigned long beatPeriod = 60000UL / bpm;  // ms por latido
  unsigned long timeInCycle = elapsed % beatPeriod;

  if (timeInCycle < 200) {
    digitalWrite(ledAzul, LOW);
  } else {
    digitalWrite(ledAzul, HIGH);
  }
}

void processAndPrintHRSpO2() {
  Serial.println("Procesando HR y SpO2 con 100 muestras...");

  // Llenar buffers con 100 muestras reales
  for (byte i = 0; i < 100; i++) {
    // Esperar a que haya datos disponibles
    while (!particleSensor.available()) {
      particleSensor.check();  // Actualiza el sensor
      delay(5);                // Pequeña espera para no bloquear excesivo
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Llamar al algoritmo con buffers llenos
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, 100, redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate);

  // Validar resultados
  if (!validHeartRate) heartRate = 0;
  if (!validSPO2) spo2 = 0;

  //data2 = "HR: " + String(heartRate) + " "
  //        + "OX: " + String(spo2);


}

// ================= FUNCIONES SENSOR =================

long getHeartBeat(long irValue) {
  return irValue;
}

byte getPresence(long irValue) {
  return irValue > 50000 ? 1 : 0;
}

float getTemperature() {
  float temp = particleSensor.readTemperature();
  return temp;
}

String getAcceleration() {
  return String(myIMU.readFloatAccelX()) + ", " + String(myIMU.readFloatAccelY()) + ", " + String(myIMU.readFloatAccelY());
}

String getGyroscope() {
  return String(myIMU.readFloatGyroX()) + ", " + String(myIMU.readFloatGyroY()) + ", " + String(myIMU.readFloatGyroZ());
}
