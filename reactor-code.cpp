// Set da tela touch screen
#include <SoftwareSerial.h>

#define rxPin 2
#define txPin 3

SoftwareSerial mySerial(rxPin, txPin);

// Set sensor de temperatura
#include <OneWire.h>
#include <DallasTemperature.h>

int temperaturePin = 5;
OneWire onewire(temperaturePin);
DallasTemperature temperatureSensor(&onewire);
DeviceAddress temperatureAddress;

// Set pwm
int pwmPin = 10;
int pwmUserValue;
double powerReadValue;
double pwmValue = 0;

// Variáveis de controle
int pgmTime = 10;
int temperature = 27; 
double pgmtimeReadValue;
int pinLevel = 6;

// Set para o reset inicial
int initialReset = 1;

// Set para a corrente
int currentPin = A0;
int currentValue;

// Endereços para a tela
#define data_time 0x10
#define data_power 0x12
#define data_temperature 0x14

// Sets para a tela
unsigned char     writePgmtime[8] = {0x5A, 0xA5, 0x05, 0x82, data_time, 0x00, 0x00, 0x00};
unsigned char       writePower[8] = {0x5A, 0xA5, 0x05, 0x82, data_power, 0x00, 0x00, 0x00};
unsigned char writeTemperature[8] = {0x5A, 0xA5, 0x05, 0x82, data_temperature, 0x00, 0x00, 0x00};
unsigned char       changePic[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};

unsigned char  readPgmtime[8] = {0x5A, 0xA5, 0x04, 0x83, 0x10, 0x00, 0x01, 0x00};
unsigned char    readPower[8] = {0x5A, 0xA5, 0x04, 0x83, 0x12, 0x00, 0x01, 0x00};
unsigned char readPoweroff[8] = {0x5A, 0xA5, 0x04, 0x83, 0x18, 0x00, 0x01, 0x00};

unsigned char buzzerOneSecond[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x7D};
unsigned char buzzerHalfSecond[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x3E};

unsigned char Buffer[9];
unsigned char receivedDataPower[8];
unsigned char receivedDataPgmtime[8];
unsigned char receivedDataPoweroff[8];

// Set para o uso do millis
unsigned long previousMillis = 0;
unsigned long currentMillis;
const long interval = 1000;
int total, resTotal;

unsigned long temperatureCheck = 0;
const long temperatureCheckInterval = 950;

unsigned long buzzerMillis;

void setup() 
{
  // Baud
  Serial.begin(9600);
  mySerial.begin(9600);

  // Sets
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(pinLevel, INPUT);

  temperatureSensor.begin();

  pinMode(pwmPin, OUTPUT);

  // Breve delay para segurar os sets
  delay(2000);
}

void loop() 
{
  // Acionamento das funções globais
  initialResetFunc();
  if (millis() - temperatureCheck >= temperatureCheckInterval)
  {
    temperatureCheck = millis();
    showTemperature();
  }
  verifyTemperatureStart();
  verifyLevel();
  readPowerFunc();
  readTime();
  validation();
  dataToDisplay();
}

void readCurrent()
{
  // Faz a leitura do valor da corrente
  currentValue = analogRead(currentPin);
  Serial.println(currentValue);
}

void verifyCurrent() 
{
  // Verificação da corrente
  if (currentValue > 736) {
    // Corrente elevada
    buzzerMillis = 0;
    buzzerMillis = millis();

    changePic[8] = 0x00;
    changePic[9] = 0x09;
    mySerial.write(changePic, 10);
    total = -1;
    screenReset();
    delay(10000);

    while (buzzerMillis > 10000) {
      mySerial.write(buzzerHalfSecond, 8);
      delay(1000);
    }
    resetPopup();
  }
}

void showTemperature()
{
  // Aquisão dos dados de temperatura
  temperatureSensor.requestTemperatures();

  if(!temperatureSensor.getAddress(temperatureAddress, 0)) {
    Serial.println("O sensor não está conectado!");
  } 
  else {
    temperature = temperatureSensor.getTempC(temperatureAddress), 1;
    writeTemperature[6] = highByte(temperature);
    writeTemperature[7] = lowByte(temperature);
    mySerial.write(writeTemperature, 8);
  }
}

void verifyTemperature()
{
  // Verifica o nível da temperatura para não passar do limite desejado
  if (temperature >= 45)
  {
    changePic[8] = 0x00; 
    changePic[9] = 0x07;
    mySerial.write(changePic, 10);
    total = -1;
    screenReset();
    
    while (temperature >= 30)
    {
      showTemperature();
      mySerial.write(buzzerHalfSecond, 8);
      delay(1000);
      Serial.println(temperature);
    }
    resetPopup();
    screenReset();
  }
}

void verifyTemperatureStart()
{
  // Verificação da temperatura logo quando o dispositivo é ligado
  if (temperature >= 30)
  {
    // Iniciando com uma temperature elevada
    changePic[8] = 0x00; 
    changePic[9] = 0x07;
    mySerial.write(changePic, 10);

    while (temperature >= 30)
    {
      mySerial.write(buzzerHalfSecond, 8);
      delay(1000);
      showTemperature();
    }
    resetPopup();
  }
}

void readPowerFunc()
{
  // Faz a leitura do endereço 0x12 e salva o valor que foi selecionado na tela touchscreen
  mySerial.write(readPower, sizeof(readPower));
  delay(100);

  if (mySerial.available() >= 8)
  {
    unsigned char receivedDataPower[8];
    int bytesreadPower = mySerial.readBytes(receivedDataPower, sizeof(receivedDataPower));

    if (bytesreadPower >= 8) 
    {
      powerReadValue = (receivedDataPower[6] << 8) | receivedDataPower[7];
    }
  }

  pwmValue = (powerReadValue / 100) * 128;
  delay(50);
}

void showPower()
{
  // Aquisição dos dados de potência
  Serial.println(pwmValue);
  analogWrite(pwmPin, pwmValue);
  Serial.println("------");
  Serial.println(pwmValue);
  
  setPower(250);

  pwmUserValue = ((pwmValue * 2) * 0.390625);

  writePower[6] = highByte(pwmUserValue);
  writePower[7] = lowByte(pwmUserValue);
  mySerial.write(writePower, 8);
}

void setPower(float freq)
{
  // Valores necessários para a conversão correta
  TCCR1A = 0x21;
  TCCR1B = 0x14;
  OCR1A = 0x7A12 / freq;
  //OCR1B = OCR1A * (duty / 255.0);
}

void readTime()
{
  // Faz a leitura do endereço 0x10 e salva o valor que foi selecionado na tela touchscreen
  mySerial.write(readPgmtime, sizeof(readPgmtime));
  delay(100);

  if (mySerial.available() >= 8)
  {
    unsigned char receivedDataPgmtime[8];
    int bytesreadPgmtime = mySerial.readBytes(receivedDataPgmtime, sizeof(receivedDataPgmtime));

    if (bytesreadPgmtime >= 8) 
    {
      pgmtimeReadValue = (receivedDataPgmtime[6] << 8) | receivedDataPgmtime[7];
    }
  }

  pgmTime = pgmtimeReadValue;
  delay(50);
}

void verifyLevel()
{
  // Verifica se tem água
  if (digitalRead(pinLevel) == HIGH)
  {
    changePic[8] = 0x00; 
    changePic[9] = 0x08;
    mySerial.write(changePic, 10);
    total = -1;
    screenReset();

    while (digitalRead(pinLevel) == HIGH)
    {
      Serial.println("Está faltando água!");
      mySerial.write(buzzerHalfSecond, 8);
      delay(1000);
    }
    resetPopup();
    screenReset();
  }
}

void validation()
{
  // Limpa o buffer
  memset(Buffer, 0, sizeof(Buffer));

  // Salva os dados da tela no buffer
  if (mySerial.available())
  {
    for (int j = 0; j < 9; j++)
    {
      Buffer[j] = mySerial.read();
    }
  }
}

void dataToDisplay()
{
  // Acionamento de funções com base no botão selecionado
  if (Buffer[0] == 0x5A)
  {
    switch (Buffer[4])
    {
      case 0x16:         
        Serial.println("Ligar!");
        showPower();
        turnOn();
        screenReset();
        // Bip de aviso:
        mySerial.write(buzzerHalfSecond, 8);
        delay(1000);
        mySerial.write(buzzerHalfSecond, 8);
        delay(1000);
        mySerial.write(buzzerOneSecond, 8);
        break;

      case 0x18:
        Serial.println("Desligar!");
        turnOff();
        break;
    }
  }
}

void turnOn()
{
  // Ação de ligar
  Serial.println("Ligou!");
  digitalWrite(LED_BUILTIN, HIGH);

  total = pgmTime * 60;
  resTotal = pgmTime;

  while (total >= 0)
  {
    currentMillis = millis();

    verifyCurrent();
    verifyTemperature();
    verifyLevel();
    delay(50);

    validation();
    if (Buffer[0] == 0x5A && Buffer[4] == 0x18)
    {
      total = -1;
      turnOff();
    }

    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;

      if (total >= 0) 
      {
        readCurrent();
        showTemperature();
        writePgmtime[6] = highByte(resTotal);
        writePgmtime[7] = lowByte(resTotal);
        mySerial.write(writePgmtime, 8);
        total--;

        if (total < 0)
        {
          changePic[8] = 0x00; 
          changePic[9] = 0x05;
          mySerial.write(changePic, 10);
        }
      }

      if (total % 60 == 0)
      {
        resTotal = resTotal - 1;
      }
    }
  }
}

void turnOff()
{
  // Ação de desligar, reseta os dados da tela no final
  changePic[8] = 0x00; 
  changePic[9] = 0x06;
  mySerial.write(changePic, 10);
  showTemperature();

  screenReset();
}

void screenReset()
{
  // Reseta os dados da tela touchscreen
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(pwmPin, 0);

  writePgmtime[7] = pgmtimeReadValue;
  mySerial.write(writePgmtime, 8);
}

void resetPopup()
{
  // Reseta os popups
  changePic[8] = 0x00; 
  changePic[9] = 0x00;
  mySerial.write(changePic, 10);
}

void initialResetFunc()
{
  // Reset que é executado somente uma vez no código
  if (initialReset < 5)
  {
    resetPopup();
    showTemperature();
    pwmValue = 26;
    showPower();
    initialReset = 10;
  }

  analogWrite(pwmPin, 0);
}