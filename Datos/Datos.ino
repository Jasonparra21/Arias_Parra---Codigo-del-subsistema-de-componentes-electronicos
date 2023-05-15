////////////////////////////////////////Librerias//////////////////////////////////////////
#include <SPI.h>		            // incluye libreria interfaz SPI
#include <SD.h>			            // incluye libreria para tarjetas SD
#include <Wire.h>               // incluye libreria para Reloj
#include "RTClib.h"             // incluye libreria para Reloj
#include <Adafruit_MAX31865.h>  // incluye librerias para PT100
#include "DFRobot_EC.h"         // incluye librerias para sensor de conductividad electrica
#include <EEPROM.h>             // incluye librerias para sensor de conductividad electrica
#include <Arduino.h>            // incluye librerias para sensor de oxigeno disuelto
//////////////////////////////////////Reloj///////////////////////////////////////////////
RTC_DS3231 rtc;
String ano;
String dia;
String mes;
String horas;
String minutos;
String segundos;
String Fecha;
String Hora;
////////////////////////////////////// SD //////////////////////////////////////////////////
#define SSpin 53		// Se selecciona el pin digital esclavo 53
File archivo;			//   Objeto archivo del tipo File
unsigned long intervaloDatalog = 1000;
unsigned long Milis=0;
///////////////////////////////////// LCD /////////////////////////////////////////////////
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Se utiliza la libreria creando el objeto LCD con los pines con sus correspondientes salidas(rs, en, d4, d5, d6, d7)
String Linea1;
String Linea2;
///////////////////////////////// Sensor de temperatura //////////////////////////////////
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13); // Pines Cs, SDI, SDO, CLK de modulo MAX31865
#define RREF      430.0 // Rref de resistencia 
#define RNOMINAL  100  // Resistencia a 0 grados
float temperatura = 0;
///////////////////////////////// Sensor de pH ///////////////////////////////////////////
#define SensorPin A0
float offsetdecalibracion = 2.0;          // Segun caliracion previa se define un offser
unsigned long int avgValue;                 // Se crea una variale para guardar el valor promedio de la retroalimentación del sensor
float pH;                                   // Se inicializa la variale pH
int buf[10],temp;                           // Esto declara una matriz variable. 
///////////////////////////////// Sensor de Conductividad electrica ///////////////////////////////////////////
#define EC_PIN A1
float voltajeCE = 0;
float valorCe = 0;
float ecValue = 0;
DFRobot_EC ec;  
///////////////////////////////// Sensor de Oxigeno disuelto ///////////////////////////////////////////
#define DO_PIN A2
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolución
#define TWO_POINT_CALIBRATION 1 // modo de calibracion de 2 puntos
#define CAL1_V (1704)    //  Voltaje    (mV) punto 1 -> temperatura alta
#define CAL1_T (40)    //  temperatura(°C) punto 1 -> temperatura alta
#define CAL2_V (732)    //  Voltaje    (mV) punto 2 -> temperatura baja
#define CAL2_T (5)     //  temperatura(°C) punto 2 -> temperatura baja
float ValorOD = 0;
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
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
/////////////////////////////////////////// ALERTAS /////////////////////////////////////////////
const byte pinBuzzer = 3;    // Se define el pin digital de salida para el buzzer
byte salida = 10;
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);				// inicializa monitor serie a 115200 bps
  lcd.begin(16, 2);           //Inicia la LCD
  thermo.begin(MAX31865_3WIRE); //Configuracion para pt100 de 3 hilos
  ec.begin();                   //Se inicia el sensor de conductividad electrica
  if (!rtc.begin()) // Comprobar si el reloj esta conectado
  {
    Serial.println("El reloj no está conectado!");
    lcd.setCursor(0,0);
    lcd.print("Reloj No conectado!"); // Se imprime en LCD mensaje
  }
  if (rtc.lostPower()) // Ajustar hora del reloj
  {
    rtc.adjust(DateTime(2022, 10, 26, 8, 56, 0));
   }
  if (!SD.begin(SSpin)) {		// Comprobar si la SD esta conectada
    Serial.println("fallo en inicializacion!");
    lcd.setCursor(0,1);
    lcd.print("Fallo en SD!"); // Se imprime en LCD mensaje
    return;					           // Se sale del setup()
  }
  Serial.println("inicializacion correcta");	// texto que confirma inicializacion   
}
////////////////////////////////////////////////////////////////////////////////////////////
void loop() {	
  ////////////////////////////////////////// Reloj /////////////////////////////////////////	  
  DateTime now = rtc.now();
  ano = now.year();
  mes = now.month();
  dia = now.day();
  horas = now.hour();
  minutos = now.minute();
  segundos = now.second();
  Fecha = ano + "/" + mes + "/" + dia;
  Hora = horas + ":" + minutos + ":" + segundos;
  ///////////////////////////////// Sensor de temperatura //////////////////////////////////
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  temperatura = thermo.temperature(RNOMINAL, RREF); //Adquisición de datos de temperatura
  temperatura = temperatura + 260;
  Serial.println(temperatura);
  /////////////////////////////// Sensor de pH ///////////////////////////////////////////
  for(int i=0;i<10;i++)       // Se obtienen 10 muestras del valor del sensor
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        // Se organizan los datos de menor a mayor
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      // Se toma el promedio de los datos del cento 
    avgValue+=buf[i];
  float ValorpH =(float)avgValue*5.0/1024/6; // Se convierte la señal analoga a milivoltios
  ValorpH=3.5*ValorpH+offsetdecalibracion;   // Se convierten los milivoltios a el valor de pH
  Serial.println(ValorpH);
  ///////////////////////////////// Sensor de Conductividad electrica ///////////////////////////////////////////
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U)  // Se define un tiempo de intervalos de 1s
  {
    timepoint = millis();
    voltajeCE = analogRead(EC_PIN)/1024.0*5000;   // Lectura del voltaje del sensor de conductividad electrica
    valorCe =  ec.readEC(voltajeCE,temperatura);  // Convierte el voltaje a conductividad electrica con una compensacion por temperatura
  }
  ec.calibration(voltajeCE,temperatura);          // Calibracion por serial CMD
  //valorCe = valorCe - 12;  
  Serial.println(valorCe);  
  ///////////////////////////////// Sensor de Oxigeno disuelto ///////////////////////////////////////////
  Temperaturet = (uint8_t)temperatura;
  ADC_Raw = analogRead(DO_PIN);                       // Lee el valor del sensor 
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;   // Ajusta los datos del sensor
  ValorOD = (readDO(ADC_Voltage, Temperaturet))/1000; // Convierte el voltaje con una compensacion por temperatura a OD
  Serial.println(ValorOD);
  /////////////////////////////////////  Datos en LCD //////////////////////////////////////////////////////
    Linea1 = String(Fecha) +" "+ String(Hora)+ " Temp: "+String(temperatura)+("C");
    Linea2 = "pH: "+String(ValorpH) +" CE: "+ String(valorCe) + " mS/cm" + " OD:" + String(ValorOD) +" mg/L";
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(Linea1);
    lcd.scrollDisplayLeft();
    lcd.setCursor(0,1);
    lcd.print(Linea2);
    lcd.scrollDisplayLeft();
    delay(300); 
  //////////////////////////////////// Alertas del sistema ///////////////////////////////////////////////
   if (temperatura > 25 ) {
     Serial.println("La temperatura del agua ha superado el rango recomendado"); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temperatura alta ("+ String(temperatura)+"C)");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);
   } else if (temperatura < 15 ) {
     Serial.println("La temperatura del agua es inferior a el rango recomendado");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temperatura baja ("+ String(temperatura)+"C)");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);    
   }
    if (ValorpH > 7.5 ) {
     Serial.println("El valor de pH del agua ha superado el rango recomendado"); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("pH alto("+ String(ValorpH)+")");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);   
     delay(6000);
   } else if (ValorpH < 6.5 ) {
     Serial.println("El valor de pH del agua del tanque es inferior a el rango recomendado");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("pH bajo("+ String(ValorpH)+")");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);
     delay(6000);      
   }
    if (valorCe > 1.5 ) {
     Serial.println("La conductividad electrica del agua ha superado el rango recomendado"); 
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("Conductividad electrica alta ("+ String(valorCe)+")");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);  
     delay(6000); 
   }
    if (ValorOD > 9 ) {
     Serial.println("El oxigeno disuelto del agua ha superado el rango recomendado"); 
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("Oxigeno disuelto alto ("+ String(ValorOD)+")");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);
     delay(6000);   
   } else if (ValorOD < 3 ) {
     Serial.println("El oxigeno disuelto del agua es inferior a el rango recomendado");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Oxigen disuelto bajo ("+ String(ValorOD)+")");
      lcd.scrollDisplayLeft();
      delay(450);
     analogWrite(pinBuzzer, salida);
     delay(6000);      
   }
  //////////////////////////////////// Escritura en SD ////////////////////////////////////////////////////  
  unsigned long Milisact = millis();
  if ((Milisact - Milis)>= (intervaloDatalog)){
    Milis = Milisact;
    archivo = SD.open("Datos.txt", FILE_WRITE);	// apertura para lectura-escritura de archivo Datos.txt
  if (archivo) {
    archivo.println(Fecha + " " + Hora + " Temperatura: " + temperatura + " " + "°C" + " pH: " + "6,7" + " Conductividad electrica: " + valorCe + " mS/cm " + "Oxigeno disuelto " + ValorOD + " mg/L");	// escritura de una linea de texto en archivo
    Serial.println("Escribiendo en archivo datos.txt...");	// texto en monitor serie
    archivo.close();				              // cierre del archivo
    Serial.println("Escritura correcta");	// texto de escritura correcta en monitor serie
  } else {
    Serial.println("Error en la apertura de datos.txt");	// texto de falla en apertura de archivo
  }   
  }
   archivo = SD.open("Datos.txt");		// apertura de archivo Datos.txt
  if (archivo) {
    Serial.println("Contenido de datos.txt:");	// texto en monitor serie
    while (archivo.available()) {		 // mientras exista contenido en el archivo
      Serial.write(archivo.read());  // lectura de a un caracter por vez
    }
    archivo.close();				// cierre de archivo
  } else {
    Serial.println("Error en la apertura de datos.txt");// texto de falla en apertura de archivo
  }   
  delay(1000);
}
