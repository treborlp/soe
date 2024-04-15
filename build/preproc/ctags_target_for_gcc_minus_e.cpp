# 1 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino"
// WIFI
# 3 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 4 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2

// LIBRERIA MQQT
# 7 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 8 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 9 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2

# 11 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 12 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
// RTC
# 14 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
ESP32Time rtc;
__attribute__((section(".rtc.data" "." "28")))

// Arduino
# 19 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2

// unix time
# 22 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2

// TAREA EN EL OTRO NUCLEO
TaskHandle_t Task1; // Se declara una tarea en el nucleo 0

// LEDS
int pin_dos = 18; // LED de mediciones y sensado
int pin_uno = 27; // LED opcional

// SLEEP
void setModemSleep(); // Configuración para bajar la velocidad frecuencia de trabajo a 10MHz en el nucleo 1 (principal)
void wakeModemSleep(); // Sube la velocidad de frecuencia de trabajo a 240MHz en el nucleo 1.

// SD (Momoria)
# 36 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 37 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 38 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
// Pines del modulo SD




SPIClass spi = SPIClass(3 /*SPI bus normally attached to pins 5, 18, 19 and 23, but can be matrixed to any pins*/); // VSPI es una variable preasignada

// PARAMETROS WIFI
const char *ssid = "VisionLab_DIGC";
const char *password = "Cr015rjhz";

// VARIABLES AWS IOT CORE



WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// DATOS A ALMECENAR
String dataMessage; // Almacenara un string con los datos concatenados de los sensores
String dataMessage1; // Almacenara promedios de las mediciones

int unsavedCount = 0; // Número de mediciones realizadas en un intervalo de tiempo (media hora)
int m = 0; //
int s = 0;

// SHT31
# 66 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 67 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
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

uint8_t loopCnt = 0; // Evaluar

// BMP280
# 82 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
# 83 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
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
int contador = 0; // Registro pulsos del balancin
bool dato = 0x1; // Por explicar
bool datoAnterior = 0x1; // Por explicar

// ENVIO RS485
# 100 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2
HardwareSerial SerialPort(2); // solo se usa dos pares de puertos seriales Tx0-Rx0 y Tx1-Rx1

// WIND DIRECCTION
int VaneValue; // Recepciona la entrada analogica (voltaje)
int Direction; // Transformar VaneValue a un rango de 0-360 grados
int CalDirection; // Por definir

float WD_promedio;
float acumulador_WD = 0;

// Winde velocity
# 112 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 2


# 113 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino"
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine //Por especificar
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine //Por especificar
float WindSpeed; // speed miles per hour
float WV_promedio;
float acumulador_WV = 0;
float WV_max = -10;
float WV_min = 100;

// IRRADIANCIA
const int sensorPin = 26; // seleccionar la entrada analogica para el sensor
int sensorValue; // variable que almacena el valor raw (0 a 1023)
float value; // variable que almacena el voltaje (0.0 a 25.0)

const int sensorPin1 = 25; // seleccionar la entrada analogica para el sensor
int sensorValue1; // variable que almacena el valor raw (0 a 1023)
float value1; // variable que almacena el voltaje (0.0 a 3-5.0)v

// UPDATE TIME
const char *ntpServer = "2.south-america.pool.ntp.org"; // Servidor NTP
const long gmtOffset_sec = -18000; // Desplazamiento GMT en segundos (esto es para GMT-5)
const int daylightOffset_sec = 0; // Offset de horario de verano

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

  Serial.println("Connecting to AWS IOT");

  while (!client.connect("arn:aws:iot:us-east-2:654654174095:thing/SOE_PALCA" /*change this*/))
  {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe("SOE_PALCA/sub");

  Serial.println("AWS IoT Connected!");
}

void publishMessage(String timeNow, int temperatura, int humedad, String windDirection, String windSpeed, String value, String value1)
{

  StaticJsonDocument<200> doc;
  doc["time_record"] = timeNow;
  doc["humidity"] = humedad;
  doc["temperature"] = temperatura;
  doc["wind_direction"] = windDirection;
  doc["wind_speed"] = windSpeed;
  doc["wind_value"] = value;
  doc["wind_value1"] = value1;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish("SOE_PALCA/pub", jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}
void loop2(void *parameter)
{
  for (;;)
  {
    digitalWrite(pin_uno, 0x1);
    dato = digitalRead(32);
    // Serial.println("\t\t\tEn núcleo -> " +  String(xPortGetCoreID()));
    // Serial.println("\t\t\tRUNNING ");
    if (dato != datoAnterior)
    {
      contador++;
    }
    datoAnterior = dato;
    //Serial.println(contador);
    delay(100);
    digitalWrite(pin_uno, 0x0);
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
      
# 223 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino" 3 4
     __null
# 223 "D:\\INAIGEM\\2024\\Laboratorio\\SOE_MEJORAS\\V3.3\\V3.3.ino"
         ,
      1,
      &Task1,
      0);

  Serial.begin(115200);
  // Conexion WIFI
  conextionWifi();
  connectAWS();

  // Configuration time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Espera a que se obtenga el tiempo
  while (time(nullptr) < 24 * 3600)
  {
    delay(100);
  }

  // Actualiza la hora en la biblioteca ESP32Time
  rtc.setTime(time(nullptr));

  // rtc.setTime(00, 30, 9, 8, 04, 2024); // rtc.setTime(s, m, h, dia, mes, año)

  SerialPort.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(32, 0x05);
  pinMode(pin_dos, 0x03);
  pinMode(pin_uno, 0x03);
  spi.begin(14, 5, 13, 15);
  if (!SD.begin(15, spi, 80000000))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  bmp.begin((0x76) /**< Alternative I2C address for the sensor. */, (0x58) /**< Default chip ID. */);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16, /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();

  // Winde velocity
  pinMode((23) /* Señal analogica de la velocidad del anemometro*/, 0x01);
  attachInterrupt(((((23) /* Señal analogica de la velocidad del anemometro*/)<40)?((23) /* Señal analogica de la velocidad del anemometro*/):-1), isr_rotation, 0x02);

  // setModemSleep();
}

void conextionWifi()
{
  // Conecta a la red Wi-Fi
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Conectando a la red Wi-Fi...");
  }

  Serial.println("Conectado a la red Wi-Fi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}
int i = 0;
void loop()
{
  // wakeModemSleep();
  s = rtc.getSecond();

  //int mili =  rtc.getMillis();

  if (s != 00){
    unsavedCount++;
    digitalWrite(pin_dos, 0x0);
    Serial.println("En núcleo -> " + String(xPortGetCoreID()));
    Serial.println((String) "unsavedCount = " + unsavedCount);
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S")); // (String) returns time with specified format
    logSDCard();
    Serial.flush(); // send immediatly
    digitalWrite(pin_dos, 0x1);
    do { ; __extension__({ unsigned __tmp; __asm__ __volatile__( "rsil	%0, " "0" "\n" : "=a" (__tmp) : : "memory" ); __tmp;}); } while (0);
    Serial.println("go to sleep");
    m = rtc.getMinute();
    Serial.println("minuto: ");
    Serial.println(m);
    /* resetear variables */
    // setModemSleep();*/

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
    logSDAver();
    //resetear variables
    unsavedCount = 0;
    contador = 0;
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
  VaneValue = analogRead(33);
  Direction = map(VaneValue, 0, 4095, 0, 360);
  CalDirection = Direction + 0; /* Por definir*/;
  if (CalDirection > 360)
    CalDirection = CalDirection - 360;
  if (CalDirection < 0)
    CalDirection = CalDirection + 360;
  Serial.print("Wind Direcction: ");
  Serial.println(CalDirection);
  delay(50);
  acumulador_WD = acumulador_WD + CalDirection;
}

void CapturaWV()
{
  Rotations = 0; // Set Rotations count to 0 ready for calculations
  delay(3000); // Wait 3 seconds to average
  do { __extension__({ unsigned __tmp; __asm__ __volatile__( "rsil	%0, " "3 /* level masked by PS.EXCM */" "\n" : "=a" (__tmp) : : "memory" ); __tmp;}); ; } while (0);
  WindSpeed = Rotations * 0.335;
  Serial.print("Wind speed  ");
  Serial.println(WindSpeed);
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
  sensorValue = analogRead(sensorPin); // realizar la lectura
  value = fmap(sensorValue, 0, 4095, 0.0, 6.5); // ENTRE 7.1 Y 7.2

  Serial.print("IO26 ");
  Serial.println(sensorValue);
  Serial.println(value); // mostrar el valor por serial

  sensorValue1 = analogRead(sensorPin1); // realizar la lectura
  value1 = fmap(sensorValue1, 0, 4095, 0.0, 6.5); // ENTRE 7.1 Y 7.2

  Serial.print("IO25 ");
  Serial.println(sensorValue1);
  Serial.println(value1); // mostrar el valor por serial
  delay(100);
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void logSDCard()
{ // SHT31

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
  dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) + "," + String(pressure_event.pressure) + "," + String(temp_event.temperature) + "," + String(CalDirection) + "," + String(WindSpeed) + "," + String(value) + "," + String(value1) + "," + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/dataloger.csv", dataMessage.c_str());

  String timeID = rtc.getTime("%y-%m-%d %H:%M:%S");

  publishMessage(timeID, (double)pressure_event.pressure, (double)temp_event.temperature, String(CalDirection), String(WindSpeed), String(value), String(value1) );
  Serial.println("ENVIADO A AWS");
}

void logSDAver()
{
  Serial.println("Sali del loop de medidas instantaneas");
  Serial.print("Save data: ");
  uUnixDate dateA = uUnixDate(rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(), rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
  Serial.println(dateA.timestamp());
  dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) + "," + String(TA_promedio) + "," + String(TA_max) + "," + String(TA_min) + "," + String(P_promedio) + "," + String(P_max) + "," + String(P_min) + "," + String(HR_promedio) + "," + String(HR_max) + "," + String(HR_min) + "," + String(WV_promedio) + "," + String(WV_max) + "," + String(WV_min) + "," + String(WD_promedio) + "," + String(contador) + "," + String(TI_promedio) + "," + "\r\n";
  Serial.println(dataMessage);
  appendFile(SD, "/Ave.csv", dataMessage.c_str());
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, "a");
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}
