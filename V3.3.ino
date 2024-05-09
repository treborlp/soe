// PROGRESS BAR
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60
// WIFI
#include "secrets.h"
#include <WiFiClientSecure.h>

// LIBRERIA MQQT
#include <PubSubClient.h>
#include "WiFi.h"
#include <ArduinoJson.h>

// #include <esp_crt_bundle.h>
// #include <ssl_client.h>
//  RTC
#include <ESP32Time.h>
ESP32Time rtc;
RTC_DATA_ATTR

// Arduino
#include <Arduino.h>

// unix time
#include "uUnixDate.h"

// TAREA EN EL OTRO NUCLEO
TaskHandle_t Task1; // Se declara una tarea en el nucleo 0

// LEDS
int pin_dos = 18; // LED de mediciones y sensado
int pin_uno = 27; // LED opcional

// SLEEP
void setModemSleep();  // Configuración para bajar la velocidad frecuencia de trabajo a 10MHz en el nucleo 1 (principal)
void wakeModemSleep(); // Sube la velocidad de frecuencia de trabajo a 240MHz en el nucleo 1.

// SD (Momoria)
#include "FS.h"
#include "SD.h"
#include "SPI.h"
// Pines del modulo SD
#define SCK 14
#define MISO 5
#define MOSI 13
#define CS 15
SPIClass spi = SPIClass(VSPI); // VSPI es una variable preasignada

// PARAMETROS WIFI
// Reemplaza con el nombre de tu red y la contraseña
const char *ssid = "wifirob";
const char *password = "12345678";
unsigned long lastAttemptTime = 0;
const int attemptInterval = 10000; // Intervalo de reconexión de 30 segundos
boolean flagConnect = false;

// VARIABLES AWS IOT CORE
#define AWS_IOT_PUBLISH_TOPIC "SOE_PALCA/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "SOE_PALCA/sub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// DATOS A ALMECENAR
String dataMessage;  // Almacenara un string con los datos concatenados de los sensores
String dataMessage1; // Almacenara promedios de las mediciones

int unsavedCount = 0; // Número de mediciones realizadas en un intervalo de tiempo (media hora)
int m = 0;            //
int currentSecond = 0;

// SHT31
#include <Wire.h>
#include "Adafruit_SHT31.h"
bool enableHeater = false;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float TA_promedio;
float acumulador_T = 0;
float TA_max = -10;
float TA_min = 200;
float HR_promedio;
float acumulador_H = 0;
float HR_max = -10;
float HR_min = 200;
float acumulador_PRE = 0;

uint8_t loopCnt = 0; // Evaluar

// BMP280
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float P_promedio;
float acumulador_P = 0;
float P_max = 100;
float P_min = 800;
float TI_promedio;
float acumulador_TI = 0;

// PLUVIO
int contador = 0;         // Registro pulsos del balancin
bool dato = HIGH;         // Por explicar
bool datoAnterior = HIGH; // Por explicar

// WIND DIRECCTION
int VaneValue;    // Recepciona la entrada analogica (voltaje)
int Direction;    // Transformar VaneValue a un rango de 0-360 grados
int CalDirection; // Por definir
#define Offset 0; // Por definir
float WD_promedio;
float acumulador_WD = 0;

// Winde velocity
#include <math.h>
#define WindSensorPin (23)                // Señal analogica de la velocidad del anemometro
volatile unsigned long Rotations;         // cup rotation counter used in interrupt routine //Por especificar
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine //Por especificar
float WindSpeed;                          // speed miles per hour
float WV_promedio;
float acumulador_WV = 0;
float WV_max = -10;
float WV_min = 100;

// IRRADIANCIA
const int sensorPin = 33; // seleccionar la entrada analogica para el sensor
int sensorValue;          // variable que almacena el valor raw (0 a 1023)
float value;              // variable que almacena el voltaje (0.0 a 25.0)
float IRRA;
const int sensorPin1 = 25; // seleccionar la entrada analogica para el sensor
int sensorValue1;          // variable que almacena el valor raw (0 a 1023)
float value1;              // variable que almacena el voltaje (0.0 a 3-5.0)v
float IRRAD2;
float Irr_promedio;
float acumulador_Irr = 0;
float Irr_max = -1000;
float Irr_min = 1000;

float Irr_promedio1;
float acumulador_Irr1 = 0;
float Irr_max1 = -1000;
float Irr_min1 = 1000;

// UPDATE TIME
const char *ntpServer = "2.south-america.pool.ntp.org"; // Servidor NTP
const long gmtOffset_sec = -18000;                      // Desplazamiento GMT en segundos (esto es para GMT-5)
const int daylightOffset_sec = 0;                       // Offset de horario de verano

void printProgress(double percentage)
{
  int val = (int)(percentage * 100);
  int lpad = (int)(percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  Serial.print("\r"); // Regresa al inicio de la línea

  String fill = "";
  String emphy = "";

 // Serial.print(val);
 // Serial.print("% [");
  for (int i = 0; i < lpad; i++)
  {
    fill = fill + "|";
  }
  for (int i = 0; i < rpad; i++)
  {
    emphy = emphy + " ";
  }

  String progressBar = String(val) + "% [" + fill + emphy + "]";
  Serial.println(progressBar);


}

void connectToWiFi(unsigned long timeout)
{
  WiFi.begin(ssid, password);
  Serial.println("CONECTANDO WIFI...");
  unsigned long startTime = millis();

  // Espera hasta que la conexión sea establecida o se agote el tiempo límite
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout)
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi CONECTADO CON EXITO");
    Serial.print("IP ASIGNADA: ");
    Serial.println(WiFi.localIP()); // Muestra la dirección IP asignada al ESP32
    connectAWS();                   // Se conecta a AWS
    flagConnect = true;
    updateTime();
  }
  else
  {
    Serial.println("\nFalló la conexión WiFi dentro del tiempo límite.");
    flagConnect = false;
  }
}

void checkWiFi(unsigned long timeout)
{

  if (WiFi.status() != WL_CONNECTED || !flagConnect)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastAttemptTime > attemptInterval)
    {
      Serial.println("RECONECTANDO WIFI...");
      connectToWiFi(timeout); // Intenta reconectarse con un tiempo límite de timeout segundos
      lastAttemptTime = currentTime;
    }
  }
}

void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("---------------------------");
  Serial.println("INICIANDO CONEXION SERVIDOR REMOTO AWS IoT");

  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected())
  {
    Serial.println("ERROR: CONEXION AWS IoT TERMINADA!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

    Serial.println("CONEXION EXITOSA AWS IoT");

}

void updateTime()
{

  Serial.println("---------------------------");
  Serial.println("INICIANDO ACTUALIZACION DE HORARIA");

  // Configuration time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Espera a que se obtenga el tiempo
  while (time(nullptr) < 24 * 3600)
  {
    delay(100);
  }

  // Actualiza la hora en la biblioteca ESP32Time
  rtc.setTime(time(nullptr));

  
  Serial.println("FECHA Y HORA DEL SISTEMA ACTUALIZADO Y LISTO");
  Serial.println("---------------------------");
}

void settingMemoryCard()
{
  Serial.println("---------------------------");
  Serial.println("INICIANDO MONTAJE MEMORIA SD");
  
  pinMode(32, INPUT_PULLUP);
  pinMode(pin_dos, OUTPUT);
  pinMode(pin_uno, OUTPUT);
  spi.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS, spi, 80000000))
  {
    Serial.println("MONTAJE MEMORIA SD FALLO");

    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("NO SE DETECTA LA MEMORIA SD");

    return;
  }
  Serial.printf("ESPACIO TOTAL DE MEMORIA: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("ESPACIO USADO DE MEMORIA: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  Serial.println("MONTAJE MEMORIA SD EXITOSO");
  Serial.println("---------------------------");

}

void settingSensorTemperatureHumidity()
{
  Serial.println("---------------------------");
  Serial.println("INICIANDO CONFIGURACION DE SENSOR TEMPERATURA Y HUMEDAD");

  sht31.begin(0x44);
  // if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
  // Serial.println("Couldn't find SHT31");
  // while (1) delay(1); }

  Serial.print("ESTADO DEL CALENTADOR: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");

  Serial.println("SENSOR TEMPERATURA Y HUMEDAD LISTO");
  Serial.println("---------------------------");

}

void settingBarometer()
{
  Serial.println("---------------------------");
  Serial.println("INICIANDO CONFIGURACION DEL BAROMETRO");
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
  Serial.println("SENSOR BAROMETRO LISTO");
  Serial.println("---------------------------");
  //bmp_temp->printSensorDetails();
}

void settingWindSensor()
{
  Serial.println("---------------------------");
  Serial.println("INICIANDO CONFIGURACION SENSOR DE VIENTO");
  // Winde velocity
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
  
  Serial.println("SENSOR DE VIENTO LISTO");
  Serial.println("---------------------------");
}

void publishMessage(String timeNow, int temperatura, int humedad, String windDirection, String windSpeed, String value, String value1, String temperaturaAmbiental, String humedadRelativa, int preciInstantanea)
{

  StaticJsonDocument<200> doc;
  doc["time_record"] = timeNow;
  doc["humidity"] = humedad;
  doc["temperature"] = temperatura;
  doc["wind_direction"] = windDirection;
  doc["wind_speed"] = windSpeed;
  doc["wind_value"] = value;
  doc["wind_value1"] = value1;
  doc["tempe_ambie"] = temperaturaAmbiental;
  doc["humedad_relativa"] = humedadRelativa;
  doc["precipitacion"] = preciInstantanea;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char *message = doc["message"];
  Serial.println(message);
}
void loop2(void *parameter)
{
  for (;;)
  {
    digitalWrite(pin_uno, HIGH);
    dato = digitalRead(32);
    // Serial.println("\t\t\tEn núcleo -> " +  String(xPortGetCoreID()));
    // Serial.println("\t\t\tRUNNING ");
    if (dato != datoAnterior)
    {
      contador++;
    }
    datoAnterior = dato;
    // Serial.println(contador);
    delay(40);
    digitalWrite(pin_uno, LOW);
  }
  vTaskDelay(10);
}

void setup()
{
  // Habilitar Nucleo 0
  xTaskCreatePinnedToCore(
      loop2,
      "Task_1",
      1000,
      NULL,
      1,
      &Task1,
      0);

  Serial.begin(115200);
  connectToWiFi(10000);

  settingMemoryCard();
  settingSensorTemperatureHumidity();
  settingBarometer();
  settingWindSensor();
}

void loop()
{
  currentSecond = rtc.getSecond();

  if (currentSecond == 10)
  {
    checkWiFi(10000); // Intenta reconectarse por 10 segundos
  }

  if (currentSecond < 57)
  {
      int value = (currentSecond * 100)/56;
      printProgress(value / 100.0);
      delay(1000); // Retardo para visualizar el progreso
  }

  if (currentSecond == 57)
  {
    unsavedCount++;
    digitalWrite(pin_dos, LOW);
    //Serial.println("En núcleo -> " + String(xPortGetCoreID()));
    //Serial.println((String) "unsavedCount = " + unsavedCount);
    //Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S")); // (String) returns time with specified format
    Serial.println("**************************");
    Serial.println("INICIANDO REGISTRO EN EN MEMORIA LOCAL");
    logSDCard();
    Serial.flush(); // send immediatly
    digitalWrite(pin_dos, HIGH);
    // sei();
    //Serial.println("go to sleep");
    m = rtc.getMinute();
    //Serial.println("minuto: ");
    //Serial.println(m);
  }

  if ((m > 58 || (m > 0 && m < 2)) && unsavedCount >= 58)
  {
    // wakeModemSleep();
    WD_promedio = acumulador_WD / unsavedCount;
    WV_promedio = acumulador_WV / unsavedCount;
    TA_promedio = acumulador_T / unsavedCount;
    HR_promedio = acumulador_H / unsavedCount;
    P_promedio = acumulador_P / unsavedCount;
    TI_promedio = acumulador_TI / unsavedCount;
    Irr_promedio = acumulador_Irr / unsavedCount;
    Irr_promedio1 = acumulador_Irr1 / unsavedCount;

    logSDAver();
    // resetear variables

    acumulador_Irr = 0;
    Irr_max = -1000;
    Irr_min = 1000;

    acumulador_Irr1 = 0;
    Irr_max1 = -1000;
    Irr_min1 = 1000;

    unsavedCount = 0;
    acumulador_PRE = 0;
    acumulador_WD = 0;
    acumulador_WV = 0;
    WV_max = -10;
    WV_min = 100;
    acumulador_T = 0;
    TA_max = -10;
    TA_min = 200;
    acumulador_H = 0;
    HR_max = -10;
    HR_min = 200;
    acumulador_P = 0;
    P_max = 100;
    P_min = 800;
    acumulador_TI = 0;
  }
  client.loop();
}

void setModemSleep()
{
  setCpuFrequencyMhz(10);
  Serial.println("MODEM MODE!");
  delay(50000);
} // (56850)  TIEMPO DE REPOSO, se probo en 56500, ahora con 56750, ahora con 56700, ahora con 56725, AHORA CON 56737(FALTA)

void wakeModemSleep()
{
  setCpuFrequencyMhz(240);
  Serial.println("full MODE!");
}

void CapturaWD()
{
  int VaneValue = analogRead(26);
  // Declaración de la variable que almacenará el valor mapeado

  // Verifica en qué rango se encuentra el valor leído y realiza el mapeo correspondiente
  if (VaneValue >= 0 && VaneValue <= 850)
  {
    Direction = map(VaneValue, 0, 850, 0, 45);
  }
  else if (VaneValue >= 851 && VaneValue <= 1090)
  {
    Direction = map(VaneValue, 851, 1090, 46, 90);
  }
  else if (VaneValue >= 1091 && VaneValue <= 1230)
  {
    Direction = map(VaneValue, 1091, 1230, 91, 135);
  }
  else if (VaneValue >= 1231 && VaneValue <= 1700)
  {
    Direction = map(VaneValue, 1231, 1700, 136, 180);
  }
  else if (VaneValue >= 1701 && VaneValue <= 1936)
  {
    Direction = map(VaneValue, 1701, 1936, 181, 225);
  }
  else if (VaneValue >= 1937 && VaneValue <= 2423)
  {
    Direction = map(VaneValue, 1937, 2423, 226, 270);
  }
  else if (VaneValue >= 2424 && VaneValue <= 2846)
  {
    Direction = map(VaneValue, 2424, 2846, 271, 315);
  }
  else if (VaneValue >= 2847 && VaneValue <= 3563)
  {
    Direction = map(VaneValue, 2847, 3563, 316, 360);
  }
  else
  {
    // Manejo de valores fuera de los rangos especificados
    Direction = -1; // Por ejemplo, asignamos -1 para indicar un valor inválido
  }
  // Imprime el valor de la dirección en el Monitor Serial
  //Serial.println(Direction);
  // Retraso para la próxima lectura
  delay(10);
  CalDirection = Direction;
  acumulador_WD = acumulador_WD + CalDirection;
}

void CapturaWV()
{
  Rotations = 0; // Set Rotations count to 0 ready for calculations
  delay(3000);   // Wait 3 seconds to average
  // cli();
  WindSpeed = Rotations * 0.335;
  //Serial.print("Wind speed  ");
  //Serial.println(WindSpeed);
  acumulador_WV = acumulador_WV + WindSpeed;
  if (WindSpeed > WV_max)
  {
    WV_max = WindSpeed;
  }
  if (WindSpeed < WV_min)
  {
    WV_min = WindSpeed;
  }
}

void isr_rotation()
{
  if ((millis() - ContactBounceTime) > 15)
  { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

void CapturaIrrad()
{

  sensorValue = analogRead(sensorPin);
  if (sensorValue >= 0 && sensorValue <= 100)
  {
    IRRA = map(sensorValue, 0, 4095, 0, 15000);
  }
  else if (sensorValue >= 101 && sensorValue <= 200)
  {
    IRRA = map(sensorValue, 0, 4095, 0, 6000);
  }
  else if (sensorValue >= 201 && sensorValue <= 450)
  {
    IRRA = map(sensorValue, 0, 4095, 0, 5000);
  }
  else if (sensorValue >= 451 && sensorValue <= 999)
  {
    IRRA = sensorValue;
  } // ENTRE 6.4 Y 6.5, restar -0.09 al  value
  else if (sensorValue >= 1000 && sensorValue <= 2500)
  {
    IRRA = map(sensorValue, 0, 4095, 0, 3500);
  }

 // Serial.print("irra1 ");
  //Serial.println(IRRA);

  sensorValue1 = analogRead(sensorPin1);          // realizar la lectura
  value1 = fmap(sensorValue1, 0, 4095, 0.0, 4.2); // ENTRE 7.1 Y 7.2

  //Serial.print("IO25 ");
 // Serial.println(sensorValue1);
 // Serial.println(value1); // mostrar el valor por serial
  IRRAD2 = value1 * 1000;
  //Serial.print("IRRAD2 ");
  //Serial.println(IRRAD2);
  delay(100);

  acumulador_Irr = acumulador_Irr + IRRA;
  acumulador_Irr1 = acumulador_Irr1 + IRRAD2;

  if (IRRA > Irr_max)
  {
    Irr_max = IRRA;
  }
  if (IRRA < Irr_min)
  {
    Irr_min = IRRA;
  }

  if (IRRAD2 > Irr_max1)
  {
    Irr_max1 = value1;
  }
  if (IRRAD2 < Irr_min1)
  {
    Irr_min1 = IRRAD2;
  }
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void logSDCard()
{ // SHT31
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  acumulador_T = acumulador_T + t;
  acumulador_H = acumulador_H + h;

  if (t > TA_max)
  {
    TA_max = t;
  }
  if (t < TA_min)
  {
    TA_min = t;
  }
  if (h > HR_max)
  {
    HR_max = h;
  }
  if (h < HR_min)
  {
    HR_min = h;
  }
  // BMP280
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  acumulador_P = acumulador_P + pressure_event.pressure;
  acumulador_TI = acumulador_TI + temp_event.temperature;
  if (pressure_event.pressure > P_max)
  {
    P_max = pressure_event.pressure;
  }

  if (pressure_event.pressure < P_min)
  {
    P_min = pressure_event.pressure;
  }

  CapturaWD();
  CapturaWV();
  CapturaIrrad();

  dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) + "," + String(t) + "," + String(h) + "," + String(pressure_event.pressure) + "," + String(temp_event.temperature) + "," + String(CalDirection) + "," + String(WindSpeed) + "," + String(IRRA) + "," + String(IRRAD2) + "," + String(contador) + "," + "\r\n";
  String payload = String(t) + "," + String(h) + "," + String(pressure_event.pressure) + "," + String(temp_event.temperature) + "," + String(CalDirection) + "," + String(WindSpeed) + "," + String(IRRA) + "," + String(IRRAD2) + "," + String(contador);

  Serial.println("INICIANDO REGISTRO EN SD LOCAL: " + dataMessage);

  appendFile(SD, "/dataloger.csv", dataMessage.c_str());

  String timeID = rtc.getTime("%Y-%m-%dT%H:%M:%S");

  if (WiFi.status() != WL_CONNECTED || !flagConnect)
  {
    Serial.println("ALERTA: SE PERDIO CONEXION WIFI");
    sendDataToBackUpFile(payload, timeID);
  }
  else
  {
    // connectAWS();
    publishMessage(timeID, (double)pressure_event.pressure, (double)temp_event.temperature, String(CalDirection), String(WindSpeed), String(IRRA), String(IRRAD2), String(t), String(h), contador);
    Serial.println("INFO: REGISTRO ENVIADO AL SERVIDOR AWS");
  }

  acumulador_PRE = acumulador_PRE + contador;

  contador = 0;
}

void sendDataToBackUpFile(String payload, String timeID)
{

  File file = SD.open("/dataloger_backup.csv", FILE_APPEND);
  if (file)
  {
    file.println(String(timeID) + "," + payload); // Asume que estás leyendo de un pin analógico
    file.close();
    Serial.println("REGISTRO EN SD LOCAL: BACKUP AWS ");

  }
  else
  {
    Serial.println("ERROR AL ABRIR ARCHIVO BACKUP AWS");
  }
}

void logSDAver()
{
  Serial.println("Sali del loop de medidas instantaneas");
  Serial.print("Save data: ");
  uUnixDate dateA = uUnixDate(rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(), rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
  Serial.println(dateA.timestamp());
  dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) + "," + String(TA_promedio) + "," + String(TA_max) + "," + String(TA_min) + "," + String(P_promedio) + "," + String(P_max) + "," + String(P_min) + "," + String(HR_promedio) + "," + String(HR_max) + "," + String(HR_min) + "," + String(WV_promedio) + "," + String(WV_max) + "," + String(WV_min) + "," + String(WD_promedio) + "," + String(acumulador_PRE) + "," + String(Irr_promedio) + "," + String(Irr_max) + "," + String(Irr_min) + "," + String(Irr_promedio1) + "," + String(Irr_max1) + "," + String(Irr_min1) + "," + String(TI_promedio) + "," + "\r\n";
  Serial.println(dataMessage);
  appendFile(SD, "/Ave.csv", dataMessage.c_str());
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("REGISTRANDO EN EL ARCHIVO: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("ERROR: ERROR AL ABRIR EL ARCHIVO DE ALMACENAMIENTO LOCAL");
    return;
  }
  if (file.print(message))
  {
    Serial.println("EL REGISTRO SE COMPLETO CON EXITO");
  }
  else
  {
    Serial.println("EL REGISTRO FALLO");
  }
  file.close();
}