#include "LSM6DS3.h"
#include "Wire.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"

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
const unsigned long samplingDuration = 20000; // 20 segundos

void setup() {
  Serial.begin(9600);

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

  particleSensor.setup(0x1F, 2, 3, 100, 411, 4096); // configuración avanzada
  particleSensor.enableDIETEMPRDY();

  if (myIMU.begin() != 0) {
    Serial.println("Fallo al inicializar LSM6DS3.");
  }
}

void loop() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentButtonState == LOW && !samplingActive) {
    // Botón presionado y no estamos muestreando: iniciar cuenta regresiva
    countdownAndStartSampling();
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

        // Imprimir datos en tiempo real mientras haya presencia
        data1 = getHeartBeat(ir) + " " +
                      getPresence(ir) + " " +
                      getTemperature() + " " +
                      getAcceleration() + " " +
                      getGyroscope();

        Serial.println(data1);

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
      Serial.println(data2);
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
  unsigned long beatPeriod = 60000UL / bpm; // ms por latido
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
      particleSensor.check(); // Actualiza el sensor
      delay(5);               // Pequeña espera para no bloquear excesivo
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Llamar al algoritmo con buffers llenos
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, 100, redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );

  // Validar resultados
  if (!validHeartRate) heartRate = 0;
  if (!validSPO2) spo2 = 0;

  data2 = "HR: " + String(heartRate) + " " 
          + "OX: " + String(spo2);
}

// ================= FUNCIONES SENSOR =================

String getHeartBeat(long irValue) {
  return "HB:" + String(irValue);
}

String getPresence(long irValue) {
  return String("PR:") + (irValue > 50000 ? "1" : "0");
}

String getTemperature() {
  float temp = particleSensor.readTemperature();
  return "TMP:" + String(temp, 2);
}

String getAcceleration() {
  return "AX:" + String(myIMU.readFloatAccelX(), 2) + " " +
         "AY:" + String(myIMU.readFloatAccelY(), 2) + " " +
         "AZ:" + String(myIMU.readFloatAccelZ(), 2);
}

String getGyroscope() {
  return "GX:" + String(myIMU.readFloatGyroX(), 2) + " " +
         "GY:" + String(myIMU.readFloatGyroY(), 2) + " " +
         "GZ:" + String(myIMU.readFloatGyroZ(), 2);
}
