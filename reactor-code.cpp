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
int pgmTime;
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

unsigned char  buzzerOneSecond[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x7D};
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

unsigned long timeCheck = 0;
const long timeCheckInterval = 1000;

unsigned long buzzerMillis;
unsigned long startBuzzerMillis;

unsigned long startDelayMillis;

void setup() 
{
  // Baud
  Serial.begin(115200);
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
  validation();
  startMenu();
  if (millis() - timeCheck >= timeCheckInterval)
  {
    initialResetFunc();
    timeCheck = millis();
    showTemperature();
    verifyTemperatureStart();
    verifyLevel();
  }
}

void readCurrent()
{
  // Faz a leitura do valor da corrente
  currentValue = analogRead(currentPin);
}

void verifyCurrent() 
{
  // Verificação da corrente
  if (currentValue > 736) {
    // Corrente elevada
    startBuzzerMillis = millis();
    buzzerMillis = 0;
    Serial.println(currentValue);
    changePic[8] = 0x00;
    changePic[9] = 0x09;
    mySerial.write(changePic, 10);
    total = -1;
    screenReset();

    while (currentValue > 736) {
      while (buzzerMillis < 10000) {
        buzzerMillis = millis() - startBuzzerMillis;
        mySerial.write(buzzerHalfSecond, 8);
        delay(1000);
      }
      readCurrent();
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
    startBuzzerMillis = millis();
    buzzerMillis = 0;
    changePic[8] = 0x00; 
    changePic[9] = 0x07;
    mySerial.write(changePic, 10);
    total = -1;
    screenReset();
    
    while (temperature >= 30)
    {
      buzzerMillis = millis() - startBuzzerMillis;
      if (buzzerMillis < 10000) {
        mySerial.write(buzzerHalfSecond, 8);
        delay(1000);
      }
      showTemperature();
    }
    resetPopup();
  }
}

void verifyTemperatureStart()
{
  // Verificação da temperatura logo quando o dispositivo é ligado
  if (temperature >= 30)
  {
    // Iniciando com uma temperature elevada
    startBuzzerMillis = millis();
    buzzerMillis = 0;
    changePic[8] = 0x00; 
    changePic[9] = 0x07;
    mySerial.write(changePic, 10);
    Serial.println(buzzerMillis);
    Serial.println("-----");

    while (temperature >= 30)
    {
      buzzerMillis = millis() - startBuzzerMillis;
      if (buzzerMillis < 10000) {
        mySerial.write(buzzerHalfSecond, 8);
        Serial.println(buzzerMillis);
        delay(1000);
      }
      showTemperature();
    }
    resetPopup();
  }
}

void readPowerFunc()
{
  // Envia o comando para ler o valor de potência da tela
  readPower[7] = 0x00;
  mySerial.write(readPower, sizeof(readPower));

  unsigned long startMillis = millis();
  bool timeout = false;

  // Aguarda até 100ms para a resposta da tela (timeout)
  while (millis() - startMillis < 100 && mySerial.available() < 8) {
    delay(10); 
  }

  if (mySerial.available() >= 8) {
    unsigned char receivedDataPower[8];
    int bytesreadPower = mySerial.readBytes(receivedDataPower, sizeof(receivedDataPower));

    if (bytesreadPower == 8) {
      // Verifica se os dados lidos são válidos
      powerReadValue = (receivedDataPower[6] << 8) | receivedDataPower[7];

      // Se o valor de potência estiver dentro do intervalo esperado (por exemplo, entre 0 e 100)
      if (powerReadValue >= 0 && powerReadValue <= 100) {
        pwmValue = (powerReadValue / 100.0) * 128; 
        Serial.print("Potência lida: ");
        Serial.println(powerReadValue);
      } else {
        Serial.println("Erro: valor de potência fora do intervalo esperado");
      }
    } else {
      Serial.println("Erro: não foi possível ler 8 bytes de dados de potência");
    }
  } else {
    timeout = true;
  }

  if (timeout) {
    Serial.println("Erro: Timeout na leitura da potência");
  }

  delay(50);
}

void showPower()
{
  // Aquisição dos dados de potência
  analogWrite(pwmPin, pwmValue);
  
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
  // Envia o comando para ler o valor de tempo na tela
  readPgmtime[7] = 0x00;
  mySerial.write(readPgmtime, sizeof(readPgmtime));

  unsigned long startMillis = millis();
  bool timeout = false;

  // Aguarda até 100ms para a resposta da tela (timeout)
  while (millis() - startMillis < 100 && mySerial.available() < 8) {
    delay(10);
  }

  if (mySerial.available() >= 8) {
    unsigned char receivedDataPgmtime[8];
    int bytesreadPgmtime = mySerial.readBytes(receivedDataPgmtime, sizeof(receivedDataPgmtime));

    if (bytesreadPgmtime == 8) {
      // Verifica se os dados lidos são válidos (por exemplo, checando um valor esperado)
      pgmtimeReadValue = (receivedDataPgmtime[6] << 8) | receivedDataPgmtime[7];
      if (pgmtimeReadValue >= 1 && pgmtimeReadValue <= 120) {
        // Se o valor de tempo estiver dentro do intervalo esperado, usa ele
        pgmTime = pgmtimeReadValue;
        Serial.print("Tempo lido: ");
        Serial.println(pgmTime);
      } else {
        Serial.println("Erro: valor de tempo fora do intervalo esperado");
      }
    } else {
      Serial.println("Erro: não foi possível ler 8 bytes de dados de tempo");
    }
  } else {
    timeout = true;
  }

  if (timeout) {
    Serial.println("Erro: Timeout na leitura do tempo");
  }

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
  delay(50);
}

void startMenu() 
{
  if (Buffer[0] == 0x5A && Buffer[4] == 0x20) {
    Serial.println("-----------------");
    do {
      readTime();
      Serial.print("Tempo: ");
      Serial.println(pgmTime);
    } while (pgmTime < 1 || pgmTime > 120);
    
    do {
      readPowerFunc();
      Serial.print("Força: ");
      Serial.println(pwmValue);
    } while (pwmValue < 0 || pwmValue > 128);

    showPower();
    turnOn();
    screenReset();
    // Bip de aviso:
    mySerial.write(buzzerHalfSecond, 8);
    delay(1000);
    mySerial.write(buzzerHalfSecond, 8);
    delay(1000);
    mySerial.write(buzzerOneSecond, 8);
    delay(1000);
    asm volatile ("  jmp 0");
  }
}

void turnOn()
{
  // Ação de ligar
  changePic[8] = 0x00; 
  changePic[9] = 0x000A;
  mySerial.write(changePic, 10);
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
    if (Buffer[0] == 0x5A && Buffer[4] == 0x25)
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
          changePic[9] = 0x04;
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

  writePower[6] = highByte(pwmUserValue);
  writePower[7] = lowByte(pwmUserValue);
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
    changePic[8] = 0x00; 
    changePic[9] = 0x02;
    mySerial.write(changePic, 10);
    initialReset = 10;
  }

  analogWrite(pwmPin, 0);
}