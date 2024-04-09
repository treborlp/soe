//RTC
#include <ESP32Time.h>
ESP32Time rtc;
RTC_DATA_ATTR 

//Arduino
#include <Arduino.h>

//unix time
#include "uUnixDate.h"

//TAREA EN EL OTRO NUCLEO
TaskHandle_t Task1; //Se declara una tarea en el nucleo 0

//LEDS
int pin_dos = 18; //LED de mediciones y sensado
int pin_uno = 27; //LED opcional

//SLEEP
void setModemSleep(); //Configuración para bajar la velocidad frecuencia de trabajo a 10MHz en el nucleo 1 (principal)
void wakeModemSleep(); //Sube la velocidad de frecuencia de trabajo a 240MHz en el nucleo 1.

//SD (Momoria)
#include "FS.h"
#include "SD.h"
#include "SPI.h"
//Pines del modulo SD
#define SCK  14 
#define MISO  5
#define MOSI  13
#define CS  15
SPIClass spi = SPIClass(VSPI); //VSPI es una variable preasignada

 

//DATOS A ALMECENAR
String dataMessage; //Almacenara un string con los datos concatenados de los sensores
String dataMessage1; // Almacenara promedios de las mediciones

int unsavedCount=0; //Número de mediciones realizadas en un intervalo de tiempo (media hora)
int m = 0; //
int s= 0;

//SHT31
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



uint8_t loopCnt = 0; // Evaluar

//BMP280
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



//PLUVIO
int contador = 0; // Registro pulsos del balancin
bool dato = HIGH; // Por explicar
bool datoAnterior = HIGH; // Por explicar

//ENVIO RS485
#include <HardwareSerial.h>
HardwareSerial SerialPort(2); // solo se usa dos pares de puertos seriales Tx0-Rx0 y Tx1-Rx1


//WIND DIRECCTION
int VaneValue;// Recepciona la entrada analogica (voltaje)
int Direction;// Transformar VaneValue a un rango de 0-360 grados
int CalDirection; // Por definir
#define Offset 0; // Por definir
float WD_promedio;
float acumulador_WD = 0;

//Winde velocity
#include <math.h>
#define WindSensorPin (23) // Señal analogica de la velocidad del anemometro
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine //Por especificar
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine //Por especificar
float WindSpeed; // speed miles per hour
float WV_promedio;
float acumulador_WV = 0;
float WV_max = -10; 
float WV_min = 100;


//IRRADIANCIA
const int sensorPin = 26;   // seleccionar la entrada analogica para el sensor
int sensorValue;      // variable que almacena el valor raw (0 a 1023)
float value;        // variable que almacena el voltaje (0.0 a 25.0)


const int sensorPin1 = 25;   // seleccionar la entrada analogica para el sensor
int sensorValue1;      // variable que almacena el valor raw (0 a 1023)
float value1;        // variable que almacena el voltaje (0.0 a 3-5.0)v




void loop2(void *parameter){
  for(;;){
      digitalWrite(pin_uno, HIGH);
dato = digitalRead(32);
    Serial.println("\t\t\tEn núcleo -> " +  String(xPortGetCoreID()));
    Serial.println("\t\t\tRUNNING ");
     if (dato != datoAnterior)
 {
  contador++;
 }
 datoAnterior = dato;
  Serial.println(contador);
  delay(100);
  digitalWrite(pin_uno, LOW);
  }
  vTaskDelay(10);
}



void setup() {
  //Habilitar Nucleo 0
  xTaskCreatePinnedToCore(
    loop2,
    "Task_1",
    1000,
    NULL,
    1,
    &Task1,
    0);

Serial.begin(115200);
rtc.setTime(00, 30, 9, 8, 04, 2024);//rtc.setTime(s, m, h, dia, mes, año)
SerialPort.begin(9600, SERIAL_8N1, 16, 17); 
pinMode(32,INPUT_PULLUP);
pinMode(pin_dos, OUTPUT);
pinMode(pin_uno, OUTPUT);
spi.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS,spi,80000000)) {
    Serial.println("Card Mount Failed");
    return;} 
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;}
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();

  //Winde velocity
pinMode(WindSensorPin, INPUT);
attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
  
  setModemSleep();    
}

void loop() {
  wakeModemSleep();
  Serial.println("checadndo segundos codigo rob");
s = rtc.getSecond();
  if (s != 0){
    delay(100);//(500)
    }
   else {
unsavedCount++;
digitalWrite(pin_dos, LOW);
  Serial.println("En núcleo -> " +  String(xPortGetCoreID()));
  Serial.println((String)"unsavedCount = "+unsavedCount);
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));   // (String) returns time with specified format
logSDCard();
Serial.flush(); // send immediatly
  digitalWrite(pin_dos, HIGH);
   sei();
   Serial.println("go to sleep");
   m = rtc.getMinute();
   Serial.println("minuto: ");
   Serial.println(m);
    /* resetear variables */
      setModemSleep();}

      if ((m > 58 || (m > 0 && m < 2))  && unsavedCount >= 58) {
    wakeModemSleep();
    WD_promedio   = acumulador_WD/unsavedCount;
    WV_promedio   = acumulador_WV/unsavedCount;
    TA_promedio   = acumulador_T/unsavedCount;
    HR_promedio   = acumulador_H/unsavedCount;
    P_promedio    = acumulador_P/unsavedCount;
    TI_promedio   = acumulador_TI/unsavedCount;   
    logSDAver();
    /* resetear variables */
    unsavedCount=0;
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
}

void setModemSleep() {
setCpuFrequencyMhz(10);
Serial.println("MODEM MODE!");
  delay(56750);} // (56850)  TIEMPO DE REPOSO, se probo en 56500, ahora con 56750, ahora con 56700, ahora con 56725, AHORA CON 56737(FALTA)

void wakeModemSleep() {
 setCpuFrequencyMhz(240);
 Serial.println("full MODE!");}

void CapturaWD() {
VaneValue = analogRead(33);
Direction = map(VaneValue, 0, 4095, 0, 360);
CalDirection = Direction + Offset;
if(CalDirection > 360)
CalDirection = CalDirection - 360;
if(CalDirection < 0)
CalDirection = CalDirection + 360;
Serial.print("Wind Direcction: "); 
Serial.println(CalDirection); 
delay (50);
acumulador_WD = acumulador_WD + CalDirection;
}


void CapturaWV () {
Rotations = 0; // Set Rotations count to 0 ready for calculations
delay (3000); // Wait 3 seconds to average
cli();
WindSpeed = Rotations * 0.335;
Serial.print("Wind speed  ");
Serial.println(WindSpeed);
acumulador_WV = acumulador_WV + WindSpeed;
if ( WindSpeed > WV_max) {WV_max = WindSpeed;}
if ( WindSpeed < WV_min  ) {WV_min  = WindSpeed;}}

void isr_rotation () {
if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();}}

void CapturaIrrad() {
  sensorValue = analogRead(sensorPin);        // realizar la lectura
  value = fmap(sensorValue, 0, 4095, 0.0, 6.5);   // ENTRE 7.1 Y 7.2

  Serial.print("IO26 ");
  Serial.println(sensorValue);
  Serial.println(value);              // mostrar el valor por serial


  sensorValue1 = analogRead(sensorPin1);        // realizar la lectura
  value1 = fmap(sensorValue1, 0, 4095, 0.0, 6.5);   // ENTRE 7.1 Y 7.2

  Serial.print("IO25 ");
  Serial.println(sensorValue1);
  Serial.println(value1);              // mostrar el valor por serial
  delay(100);
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;}


void logSDCard()
{ //SHT31

  //BMP280
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

acumulador_P = acumulador_P + pressure_event.pressure;
acumulador_TI = acumulador_TI + temp_event.temperature;
      if ( pressure_event.pressure > P_max) {
        P_max = pressure_event.pressure;}

      if ( pressure_event.pressure < P_min) {
         P_min = pressure_event.pressure;}
          


 
  CapturaWD(); 
  CapturaWV();  
  CapturaIrrad();
  dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) +"," + String(pressure_event.pressure) + "," + String(temp_event.temperature) + "," +  String(CalDirection) + "," +  String(WindSpeed) + ","  +  String(value) + "," +  String(value1) + ","  + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/dataloger.csv", dataMessage.c_str());}
  
void logSDAver()
{ Serial.println("Sali del loop de medidas instantaneas");
  Serial.print("Save data: ");
uUnixDate dateA = uUnixDate(rtc.getYear(), rtc.getMonth()+1, rtc.getDay(), rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
Serial.println(dateA.timestamp());
 dataMessage = String(rtc.getTime("%B %d %Y %H:%M:%S")) + "," + String(TA_promedio) + "," + String(TA_max)+ "," + String(TA_min) + "," + String(P_promedio) + "," + String(P_max)+ "," + String(P_min) + "," + String(HR_promedio) + "," + String(HR_max)+ "," + String(HR_min) + "," + String(WV_promedio)+ "," + String(WV_max)+ "," + String(WV_min)+  "," + String(WD_promedio) + "," + String(contador) + "," + String(TI_promedio) + "," + "\r\n";
 Serial.println(dataMessage);
appendFile(SD, "/Ave.csv", dataMessage.c_str());}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;}
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();}
