//Keypad Library
#include <Keypad.h>

//I2C-Serial Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//TSL Sensor 
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#include <math.h>

//********************* KEYPAD VARIABLES *********************//
const byte ROWS = 4;  //four rows
const byte COLS = 4;  //four columns
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 12, 11, 10, 9 };  //connect to the row pinouts of the keypad
byte colPins[COLS] = { 8, 7, 6, 5 };     //connect to the column pinouts of the keypad

bool interruptEnabled = true;

//CREATE OBJECT OF THE KEYPAD
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Interrupt button
#define BUTTON_PIN 18

//********************* I2C-Serial VARIABLES *********************//
LiquidCrystal_I2C lcd(0x27, 16, 2);

//********************* ULTRASONIC SENSOR VARIABLES *********************//
const int trig = 32;
const int echo = 30;
const int greenLED = 28;
const int blueLED = 26;
const int yellowLED = 24;
const int redLED = 22;
long duration, distanceCm;

//********************* LDR SENSOR VARIABLES *********************//
float LDRAnalogVal;
float duty;
#define motorOut 3
#define LDRInput A0

//********************* CAMERA ISO VARIABLES *********************//
float resistance = 0.0;
float lux = 0.0;
float ISO = 0.0;
float voltage = 0.0;

//********************* INTERRUPT VARIABLES *********************//
int flag = 0;
char key;

  //******************** TSL SENSOR VARIABLES *********************//
//Definicoes do sensor TSL2561
Adafruit_TSL2561_Unified tsl=Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT,12345);

int valor = 0;
String str;
int tamanho;
sensors_event_t event;
float lux_new = 0;

void setup() {
  //LCD Setup Functions
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  //Ultrasonic Sensor Setup Function
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  //LDR Sensor Setup Function
  pinMode(motorOut, OUTPUT);
  pinMode(LDRInput, INPUT);

  analogWrite(motorOut, 0);  //safety speed reset of the motor


  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), keypadInterrupt, HIGH); // Attach interrupt to the PUSH BUTTON

  //TSL sensor Setup Function
  configureSensor();

  Serial.begin(9600);
}


void loop() {
  while (flag == 0) {
    Serial.println("Enter a number");

    Wire.beginTransmission(0x27);
    delay(60);
    lcd.clear();
    delay(5);
    lcd.setCursor(2, 0);
    lcd.print("PRESS NUMBER");

    lcd.setCursor(6, 1);
    lcd.print("1 OR 2");
    Wire.endTransmission();
    key = keypad.waitForKey();
    if (key) {
      Serial.print("Key: ");
      Serial.println(key);
      flag = 1;
    }
  }

  if (flag == 1) {
    Wire.beginTransmission(0x27);
    lcd.clear();
    delay(5);
    Wire.endTransmission();
    

    if (key == 49) {

      UltrasonicLoop();

      Wire.beginTransmission(0x27);
      lcd.setCursor(0, 0);
      lcd.print("UltraSonic work");
      lcd.setCursor(0, 1);
      lcd.print("Distance = " + String(distanceCm) + "cm");
      Serial.println("FLAG = " + String(flag));
      delay(500);
       Wire.endTransmission();
      
    }


    if (key == 50) {
      Wire.beginTransmission(0x39);
      LDRLoop();
      //TSLLoop();
     
      
      Wire.beginTransmission(0x27);
      lcd.setCursor(0, 0);
      lcd.print("LDR ISO = " + String(ISO));
      lcd.setCursor(0, 1);
      lcd.print("TSL=" + String(valor) + ", LDR=" + String(lux));
      delay(500);
      Wire.endTransmission();
      //Serial.print(" Duty = " + String(duty));
    }
  }
}


void UltrasonicLoop() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distanceCm = duration * 0.034 / 2;

  if (distanceCm < 25) {  // below One Quarter
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);
  } else if (distanceCm < 50 && distanceCm > 25) {  // below half level, but more than a quarter
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);
  } else if (distanceCm < 75 && distanceCm > 50) {  // below Three quarters
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, LOW);
  } else if (distanceCm < 100 && distanceCm > 75){  //Above Three quarters
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, HIGH);
  }

  else if (distanceCm > 100){
    distanceCm = 100;
  }
}

void LDRLoop() {
  LDRAnalogVal = analogRead(LDRInput);
  duty = map(LDRAnalogVal, 0, 1023, 0, 255);
  analogWrite(motorOut, duty);

  Serial.println("motor");
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);

  Serial.println(LDRAnalogVal);
  
  delay(60);
  voltage = ((5/1023.0) * LDRAnalogVal);
  
  Serial.println("voltage = " + String(voltage));
  float voltageDiv = voltage /3;
  Serial.println("voltageDiv = " +String(voltageDiv));
  resistance = (10000) * (1 / (0.974 - (voltageDiv/ 5)) - 1);

  Serial.println("Resistance = " + String(resistance));
  lux = ((6*pow(10,6)) * (pow(resistance,-1.301)));
  Serial.println("LDR LUX = " + String(lux));
  ISO = (-0.79 * lux) + 1679;

  TSLLoop();
}


void configureSensor(void)
{
  
  tsl.enableAutoRange(true);         
  
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);    
}
void TSLLoop() {
  
  tsl.getEvent(&event);
  //Mostra os resultados no serial monitor
  //Intensidade da Luz medida em Lux
  if (event.light)
  {
    valor = event.light;
   // Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    Serial.println("Sensor overload");
  }
  delay(250);
}

void keypadInterrupt() {
   analogWrite(motorOut, 0);
  flag = 0;
}
