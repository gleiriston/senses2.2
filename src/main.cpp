
#include <Arduino.h>
#include <Wire.h>
#include <sstream>
#include <stdio.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include "ICM42688.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <EEPROM.h>

// UUIDs
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define DHTDATA_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define DEFAULT_DELAY_STATUS_MS 1000
#define DELAY_TO_READ_US 180

#define DEVICE_DELAY_LOOP_FAST_MS 2
#define DEVICE_DELAY_LOOP_NORMAL_MS 100
#define DEVICE_DELAY_LOOP_LOW_MS 500
#define DEVICE_RESET_MS 1000
#define DEVICE_SAMPLE_TEST 5
//  BATERIA
// Endereço do MAX17048
#define ADDRESS_MAX17048 0x36
// Registradores do MAX17048
#define VCELL 0x02
#define SOC 0x04
#define MODE 0x06
#define VERSION 0x08
#define HIBRT 0x0A
#define CONFIG 0x0C
#define VALRT 0x14
#define CRATE 0x16
#define VRESET_ID 0x18
#define STATUS 0x1A
#define CMD 0xFE

#define fator_tensao_ADC 1.25E-03
#define fator_valor_SOC 3.9065E-03

// INA
#define INA_ADDRESS (uint8_t)0x40
#define INA_REGISTER_SIZE 2
#define INA_CONFIGURATION_REGISTER_ADDRESS (uint8_t)0x00
#define INA_BUS_VOLTAGE_REGISTER_ADDRESS (uint8_t)0x02
#define INA_READING_TRIGGER (uint16_t)0x0002
#define INA_DEFAULT_DATA_VALUE (uint16_t)0x4000

#define INA_SDA_PIN 21
#define INA_SCL_PIN 22
#define INA_I2C_SPEED (uint32_t)400000

#define INSOLE_CHANNELS (uint8_t)16
#define INSOLE_MIN_VALUE (uint16_t)100
#define MUX_SELECTORS 4

#define BLE_MTU_SIZE 100

// Pin Mapping - Analog Input
#define SIG 2

// Pin Mapping - Digital Output
#define S0 27
#define S1 26
#define S2 33
#define S3 32
#define EN 25

#define LED_STATUS 17
#define LED_ON digitalWrite(LED_STATUS, LOW)
#define LED_OFF digitalWrite(LED_STATUS, HIGH)
#define STATUS_STACK_SIZE 3000
#define STATUS_TASK_PRIORITY 2
#define STATUS_TASK_CORE 1

#define SERIAL_BAUD_RATE 115200

#define MEM_SIZE 100

#define MEM_FEET_ADDRESS 1

#define HL7_PAR_SEPARATOR char('~');

/**
 * @brief Types, Enums
 *
 */

typedef enum
{
  smContinuous = 0,
  smSample
} SendMode_t;

typedef struct
{
  uint8_t connected;
  uint8_t active;
  uint8_t sync;
  uint8_t pendingUpdate;
  SendMode_t sendMode;
  uint8_t sampleCounter;
  uint32_t delayLoop;
  uint32_t delayStatus;
} Device_t;

typedef struct
{
  uint16_t avg;
  uint16_t vbusct;
  uint16_t vshct;
  uint16_t mode;

  uint16_t data;

  uint8_t d1;
  uint8_t d2;

} INASetup_t;

typedef struct
{
  float x;
  float y;
  float z;
} ImuVector_t;

typedef struct
{
  ImuVector_t acc;
  ImuVector_t gyro;
  float temp;
} Imu_t;

typedef enum
{
  ftNone = char('N'),
  ftLeft = char('L'),
  ftRight = char('R')
} Feet_t;

typedef struct
{
  Feet_t feet;
  std::string application;
  std::string name;
} Insole_t;

/**
 * @brief Constants
 *
 */

/**
 * @brief Function Prototypes
 *
 */
// Variáveis para armazenar os dados da bateria
float VCELL_bateria = 0;
float SOC_bateria = 0;

// Funções para leitura e escrita no MAX17048
void Write_MAX17048(uint8_t addressRegister, uint16_t data);
uint16_t Read_MAX17048(uint8_t addressRegister);
String VCELL_SOC = "";

void deviceSetup(void);
void deviceUpdate(void);
void deviceProcessIncomingData(std::string pMessage);
void deviceSendStatus(void);
void deviceReadConfig(void);
void deviceSaveConfig(void);
void deviceSetFeet(uint8_t pFoot);
void deviceRestart(void);

void inaInit(void);
void imuInit(void);
void bleInit(void);

void readAndTransmit(void);

void readInsole(void);
int16_t readInsoleChannel(uint8_t);

void statusInit();
static void statusTask(void *pvParameters);
TaskHandle_t statusTaskHandle = nullptr;

std::string getDateTime();

/**
 * @brief Global Variables
 *
 */
ICM42688 IMU(SPI, 5);

BLEServer *pServer = NULL;

BLEService *pService = NULL;

BLECharacteristic *pCharacteristic = NULL;

INASetup_t inaSetup;

Device_t device;

Insole_t insole;

// o pino EN do mux vai no GND

int selectorMatrix[MUX_SELECTORS] = {S3, S2, S1, S0};

float insoleChannelsValues[INSOLE_CHANNELS];

// Binary Values to Multiplex Selectors
byte multiplexMatrix[INSOLE_CHANNELS][MUX_SELECTORS] = {
    {0, 0, 0, 0}, // 0  em decimal
    {0, 0, 0, 1}, // 1  em decimal
    {0, 0, 1, 0}, // 2  em decimal
    {0, 0, 1, 1}, // 3  em decimal
    {0, 1, 0, 0}, // 4  em decimal
    {0, 1, 0, 1}, // 5  em decimal
    {0, 1, 1, 0}, // 6  em decimal
    {0, 1, 1, 1}, // 7  em decimal
    {1, 1, 1, 1}, // 15 em decimal 1111
    {1, 1, 1, 0}, // 14 em decimal 1110
    {1, 1, 0, 1}, // 13 em decimal 1101
    {1, 1, 0, 0}, // 12 em decimal 1100
    {1, 0, 1, 1}, // 11 em decimal 1011
    {1, 0, 1, 0}, // 10 em decimal 1010
    {1, 0, 0, 1}, // 9  em decimal 1001
    {1, 0, 0, 0}, // 8  em decimal 1000
};

/**
 * @brief BLE Classes and Methods
 *
 */
class MyServerCallbacks : public BLEServerCallbacks
{

  /*
  In BLE, the amount of data you can send in a single packet is determined by
  the Maximum Transmission Unit (MTU) size. By default, the BLE specification
   sets an MTU size of 23 bytes. After accounting for the 3-byte header,
   4-byte MIC (if encryption is enabled), and 4-byte L2CAP header,
   you're left with 20 bytes for the ATT payload. If you're seeing a limit of
   12 characters, it's possible that there are additional headers
   or structures in your specific implementation
  */

  // void onConnect(BLEServer *pServer)
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) // new method to set MTU
  {
    uint16_t _connId;
    Serial.printf("[INFO] Client Connected!\n");
    // Serial.printf("[INFO] Current BLEDevice MTU Size : %d\n", BLEDevice::getMTU());
    device.connected = true;
    // Change MTU
    // BLEDevice::setMTU(BLE_MTU_SIZE);
    // _connId = param->connect.conn_id;               // Get the connection ID from the callback parameter
    // pSServer->updatePeerMTU(BLE_MTU_SIZE, _connId); // Request a MTU siz for this connection

    BLEDevice::startAdvertising();
    // Serial.printf("[INFO] New BLEDevice MTU Size : %d\n", BLEDevice::getMTU());
  }

  void onDisconnect(BLEServer *pServer)
  {
    Serial.printf("[INFO] Client Disconnected!\n");
    device.connected = false;
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    unsigned int _counter;

    // point to characteristic value
    std::string _rxMessage = characteristic->getValue();
    // check if received value has length greater than zero
    if (_rxMessage.length() > 0)
    {
      unsigned int _length = _rxMessage.length();
      Serial.printf("[INFO] Received message: ");
      for (_counter = 0; _counter < _length; _counter++)
      {
        Serial.printf("%02X ", _rxMessage[_counter]);
      }
      Serial.println();

      deviceProcessIncomingData(_rxMessage);
    }
    std::string rxHora = characteristic->getValue();
    if (rxHora.length() > 0)
    {
      int32_t unixTime = strtol(rxHora.c_str(), nullptr, 10);
      unixTime -= 3 * 3600; // Subtract 3 hours
      timeval epoch = {
          unixTime,
      };
      settimeofday((const timeval *)&epoch, nullptr);
    }
  }
};

/**
 * @brief Setup Section
 *
 */

void setup()
{

  Serial.begin(SERIAL_BAUD_RATE);

  deviceSetup();
  statusInit();
  inaInit();
  imuInit();
  bleInit();

  Wire1.begin(4, 16);
  Wire1.setClock(400000);

  delay(5000);
}

/**
 * @brief Loop Section
 *
 */

void loop()
{
  boolean _process; // control flat to reading insole and imu and transmit to BLE

  _process = false;

  if (device.connected)
  {
    if (device.pendingUpdate)
    {
      device.pendingUpdate = false;
    }
    else
    {
      if (device.sync)
      {
        if (device.active)
        {
          LED_OFF;
          // when sending continuously
          if (device.sendMode == smContinuous)
          {
            _process = true;
          }
          else
          {
            // when sending sampling and stop
            if (device.sampleCounter > 0)
            {
              device.sampleCounter--;
              _process = true;
            }
          }
        }
      }
      else
      {
        deviceSendStatus();
      }
    }

    if (_process)
    {
      _process = false;
      readAndTransmit();
    }
  }
}

void app_main()
{
  setup();
  while (1)
  {
    loop();
  }
}

void inaInit(void)
{
  Wire.begin(INA_SDA_PIN, INA_SCL_PIN);

  Wire.setClock(INA_I2C_SPEED);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(EN, LOW); // Multiplexer ever enabled

  pinMode(LED_STATUS, OUTPUT);
  LED_OFF;

  inaSetup.avg = 0;
  inaSetup.vbusct = 0;
  inaSetup.vshct = 0;
  inaSetup.mode = INA_READING_TRIGGER;

  inaSetup.data = INA_DEFAULT_DATA_VALUE | (inaSetup.avg << 9) | (inaSetup.vbusct << 6) | (inaSetup.vshct << 3) | inaSetup.mode;

  inaSetup.d1 = (inaSetup.data & 0xFF00) >> 8;
  inaSetup.d2 = inaSetup.data & 0x00FF;
}
Imu_t imuData = {0};
void imuInit(void)
{
  imuData.acc.x = 0;
  imuData.acc.y = 0;
  imuData.acc.z = 0;
  imuData.gyro.x = 0;
  imuData.gyro.y = 0;
  imuData.gyro.z = 0;
  imuData.temp = 0;
  // start communication with IMU
  int status = IMU.begin();
  if (status < 0)
  {
    Serial.println("[INFO] IMU initialization unsuccessful");
    Serial.println("[INFO] Check IMU wiring or try cycling power");
    Serial.print("[INFO] Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }

  // setting the aaccelerometer full scale range to +/-8G
  IMU.setAccelFS(ICM42688::gpm8);

  // setting the gyroscope full scale range to +/-50deg/s
  IMU.setGyroFS(ICM42688::dps500);

  // set output data rate to 12.5Hz
  IMU.setAccelODR(ICM42688::odr12_5);

  // set output data rate to 12.5Hz
  IMU.setGyroODR(ICM42688::odr12_5);
}

void bleInit(void)
{
  // Create the BLE Device
  BLEDevice::init(insole.name.c_str()); // Give it a name

  // Configura o dispositivo como Servidor BLE
  pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallbacks());

  // Cria o servico UART
  pService = pServer->createService(SERVICE_UUID);

  // Cria uma Característica BLE para envio dos dados
  pCharacteristic = pService->createCharacteristic(
      DHTDATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  // cria uma característica BLE para recebimento dos dados
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new CharacteristicCallbacks());

  // Inicia o serviço
  pService->start();

  // Inicia a descoberta do ESP32
  pServer->getAdvertising()->start();
}

void readAndTransmit(void)
{
  static uint16_t packet = 1;
  uint8_t reading_index = 1;
  uint32_t sendTimer = 0;
  bool sensorActivated = false;

  sendTimer = millis();

  // read insole
  readInsole();
  {
    // Adiciona os dados da bateria à string
    // Supondo que VCELL_bateria e SOC_bateria sejam do tipo double
    double VCELL_bateria_atual = fator_tensao_ADC * (Read_MAX17048(VCELL) >> 4);
    double SOC_bateria_atual = fator_valor_SOC * Read_MAX17048(SOC);

    // Definir uma variável para controlar a primeira notificação
    static bool primeira_notificacao = true;
    // Verificar se as mudanças atendem aos critérios para enviar uma notificação
    bool enviar_notificacao = false;
    if (std::abs(VCELL_bateria_atual - VCELL_bateria) >= 1.0 || std::abs(SOC_bateria_atual - SOC_bateria) >= 1.0 || std::abs(SOC_bateria_atual - SOC_bateria) >= 0.01)
    {
      enviar_notificacao = true;
      VCELL_bateria = VCELL_bateria_atual;
      SOC_bateria = SOC_bateria_atual;
    }

    // Se houver uma mudança significativa ou é a primeira notificação, enviar a notificação
    if (enviar_notificacao || primeira_notificacao)
    {
      // Converter os valores para string com duas casas decimais
      std::string strVCELL = std::to_string(VCELL_bateria);
      std::string strSOC = std::to_string(SOC_bateria);
      size_t found = strVCELL.find('.');
      if (found != std::string::npos && strVCELL.size() > found + 3)
      {
        strVCELL = strVCELL.substr(0, found + 3); // Manter apenas duas casas decimais
      }
      found = strSOC.find('.');
      if (found != std::string::npos && strSOC.size() > found + 3)
      {
        strSOC = strSOC.substr(0, found + 3); // Manter apenas duas casas decimais
      }

      // Concatenar os valores formatados à string str
      std::string str = "OBX|CELL: " + strVCELL + "~SOC:" + strSOC + "%" + getDateTime() + "\r";
      pCharacteristic->setValue(str.c_str());
      pCharacteristic->notify();
      primeira_notificacao = false;
      str += "\r";
      vTaskDelay(pdMS_TO_TICKS(device.delayLoop));
    }
    // Leitura dos dados do sensor ICM42688
    IMU.getAGT(); // Obtém aceleração, giroscópio e temperatura

    // Formata os dados para transmissão
    char imuData[256];
    snprintf(imuData, sizeof(imuData),
             "IMU: Accel: X=%.2f Y=%.2f Z=%.2f, Gyro: X=%.2f Y=%.2f Z=%.2f, Temp=%.2f",
             IMU.accX(), IMU.accY(), IMU.accZ(),
             IMU.gyrX(), IMU.gyrY(), IMU.gyrZ(),
             IMU.temp());

    // Transmitting the IMU
    std::string str = "OBX|" + std::to_string(reading_index++) + "|ST|IMU|";
    str += std::to_string(IMU.accX()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.accY()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.accZ()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.gyrX()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.gyrY()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.gyrZ()) + HL7_PAR_SEPARATOR;
    str += std::to_string(IMU.temp());
    str += "\r";
    pCharacteristic->setValue((char *)str.c_str());
    pCharacteristic->notify();
    vTaskDelay(pdMS_TO_TICKS(device.delayLoop));
  }

  Serial.printf("Reading Elapsed Time: %lu\n", millis() - sendTimer);

  sendTimer = millis();
  for (int _channel = 0; _channel < INSOLE_CHANNELS; _channel++)
  {
    if (insoleChannelsValues[_channel] > 0.560)
    { // Verifica se é maior que 0.515
      sensorActivated = true;
      break;
    }
  }

  if (sensorActivated)
  {
    char valueBuffer[20]; // Buffer para armazenar o valor formatado
    std::string str = "MSH|^~\\&|";
    str += insole.application.c_str();
    str += "|II|MP|RD|" + getDateTime() + "|MSG^TX|MSG" + std::to_string(packet) + "\r";
    packet++;
    pCharacteristic->setValue((char *)str.c_str());
    pCharacteristic->notify();

    // Transmitting the Insole
    str = "OBX|" + std::to_string(reading_index) + "|ST|PS|";
    reading_index++;

    for (int _channel = 0; _channel < INSOLE_CHANNELS; _channel++)
    {
      if (insoleChannelsValues[_channel] > 0.600)
      { // Inclui na transmissão se for maior que 0.515
        // Formata o valor flutuante com três casas decimais
        sprintf(valueBuffer, "~S%d:%.3fk", _channel + 1, insoleChannelsValues[_channel]);
        str += valueBuffer;
        if (_channel < (INSOLE_CHANNELS - 1) && insoleChannelsValues[_channel + 1] > 0.600)
        {
          str += HL7_PAR_SEPARATOR;
        }
      }
    }
    str += "\r";
    pCharacteristic->setValue((char *)str.c_str());
    pCharacteristic->notify();
  }
  vTaskDelay(pdMS_TO_TICKS(device.delayLoop));
}

// Read Insole
const float CALIBRATION_CONSTANT = (8.0 - 0.5) / 2046.0; // Calibração para o intervalo de 500g a 8kg

void readInsole()
{
  for (int _counterChannel = 0; _counterChannel < INSOLE_CHANNELS; _counterChannel++)
  {
    for (int _counterSelector = 0; _counterSelector < MUX_SELECTORS; _counterSelector++)
    {
      digitalWrite(selectorMatrix[_counterSelector], multiplexMatrix[_counterChannel][_counterSelector]);
    }

    // Lê o canal atual da palmilha
    int16_t rawValue = readInsoleChannel(INA_ADDRESS);

    // Modificação: Convertendo para quilogramas usando a constante de calibração
    // e armazenando como um valor fracionado (float)
    float calibratedValue = static_cast<float>(rawValue) * CALIBRATION_CONSTANT + 0.5;

    // Armazenar o valor calibrado no array de valores da palmilha (que deve ser float)
    insoleChannelsValues[_counterChannel] = calibratedValue;
  }
}

// Read Insole Channel
int16_t readInsoleChannel(uint8_t pInaAddress)
{
  int16_t _insoleChannelValue = 0;
  uint8_t _index = 0;
  uint8_t _registerSize = INA_REGISTER_SIZE;
  uint8_t _registerBuffer[INA_REGISTER_SIZE] = {0, 0};

  // point to INA CONFIGURATION register
  Wire.beginTransmission(pInaAddress);
  Wire.write(INA_CONFIGURATION_REGISTER_ADDRESS); // Aponta para o registrador de configuração
  Wire.write(inaSetup.d1);                        // 1 amostras e tempo de amostragem de 588us
  Wire.write(inaSetup.d2);                        // Modo de único disparo tensao de barra
  Wire.endTransmission();

  // delay to read data
  delayMicroseconds(DELAY_TO_READ_US);

  // point to INA BUS VOLTAGE register
  Wire.beginTransmission(pInaAddress);
  Wire.write(INA_BUS_VOLTAGE_REGISTER_ADDRESS);
  Wire.endTransmission();

  // read the INA BUS VOLTAGE register
  _index = 0;
  Wire.requestFrom(pInaAddress, _registerSize);
  while (Wire.available() && _index < _registerSize)
  {
    _registerBuffer[_index] = Wire.read();
    _index++;
  }

  // Compose read input channel value
  _insoleChannelValue = (_registerBuffer[0] << 8) | _registerBuffer[1];

  return _insoleChannelValue;
}

std::string getDateTime()
{
  time_t now;
  struct tm timeinfo;
  char buffer[32];
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(buffer, sizeof(buffer), "%d/%m/%Y-%H:%M:%S", &timeinfo);

  return buffer;
}

void deviceSetup(void)
{
  device.connected = false;
  device.active = false;
  device.sendMode = smContinuous;
  device.sampleCounter = 0;

  device.delayLoop = DEVICE_DELAY_LOOP_FAST_MS;
  device.delayStatus = DEFAULT_DELAY_STATUS_MS;
  device.sync = false;
  device.pendingUpdate = false;

  deviceReadConfig();
}

void deviceUpdate(void)
{
}

void deviceRestart(void)
{
  Serial.printf("[ALERT] Device Restart\n");
  // vTaskSuspend(statusTaskHandle);
  LED_OFF;
  device.active = false;
  // pService->stop();
  // vTaskDelay(pdMS_TO_TICKS(DEVICE_RESET_MS));
  ESP.restart();
}

void deviceReadConfig(void)
{
  Feet_t _value;

  insole.name = "SENSESHOES-L";

  EEPROM.begin(MEM_SIZE);

  _value = (Feet_t)EEPROM.read(MEM_FEET_ADDRESS);

  if ((_value != ftLeft) && (_value != ftRight) && (_value != ftNone))
  {
    _value = ftNone;
    EEPROM.write(MEM_FEET_ADDRESS, _value);
    EEPROM.commit();
  }
  insole.name += _value;

  insole.application = "S";
  insole.application += _value;

  Serial.printf("Dispositivo: %s\n", insole.name.c_str());
}

void deviceSaveConfig(void)
{
  EEPROM.begin(MEM_SIZE);
  EEPROM.write(MEM_FEET_ADDRESS, insole.feet);
  EEPROM.commit();
  deviceRestart();
}

void deviceSetFeet(Feet_t pFeet)
{
  if ((pFeet == ftLeft) || (pFeet == ftRight) || (pFeet == ftNone))
  {
    insole.feet = pFeet;
    deviceSaveConfig();
  }
}

void deviceSendStatus(void)
{
}

void deviceProcessIncomingData(std::string pMessage)
{
  std::string _message = "";

  Serial.printf("[INFO] Received Message: %s\n", pMessage.c_str());

  for (int i = 0; i < pMessage.length(); i++)
  {
    if (pMessage[i] != '\n' && pMessage[i] != '\r')
    {
      _message += pMessage[i];
    }
  }

  // MSH|^~\\&|SA|SF|RA|RF|yyymmddhhnnss|CMD^ID|MSG0001 \r

  // TODO Parsing data
  if (_message == "CT_RESET")
  {
    // Gracefull shutdown
    deviceRestart();
  }

  if (_message == "CC_FEET_LEFT")
  {
    deviceSetFeet(ftLeft);
  }

  if (_message == "CC_FEET_RIGHT")
  {
    deviceSetFeet(ftRight);
  }

  if (_message == "CC_FEET_NONE")
  {
    deviceSetFeet(ftNone);
  }

  if (_message == "CT_START")
  {
    if (device.sync)
    {
      device.active = true;
      LED_OFF;
      vTaskSuspend(statusTaskHandle);
    }
  }

  if (_message == "CT_STOP")
  {
    device.active = false;
    vTaskResume(statusTaskHandle);
  }

  if (_message == "CS_ON")
  {
    device.sync = true;
  }

  if (_message == "CS_OFF")
  {
    device.sync = false;
    device.active = false;
  }

  if (_message == "PD_LOW")
  {
    device.delayLoop = DEVICE_DELAY_LOOP_LOW_MS;
  }

  if (_message == "PD_NORMAL")
  {
    device.delayLoop = DEVICE_DELAY_LOOP_NORMAL_MS;
  }

  if (_message == "PD_FAST")
  {
    device.delayLoop = DEVICE_DELAY_LOOP_FAST_MS;
  }

  if (_message == "PM_CONT")
  {
    device.sendMode = smContinuous;
    device.sampleCounter = 0;
  }

  if (_message == "PM_SAMPLE")
  {
    device.sendMode = smSample;
    device.sampleCounter = DEVICE_SAMPLE_TEST;
  }

  device.pendingUpdate = true;
}
void Write_MAX17048(uint8_t addressRegister, uint16_t data)
{

  uint8_t D[3] = {0, 0, 0};

  D[0] = addressRegister;
  D[1] = (data & 0xFF00) >> 8;
  D[2] = (data & 0x00FF);

  Wire1.beginTransmission(ADDRESS_MAX17048);
  Wire1.write(D, sizeof(D));
  Wire1.endTransmission();
}

uint16_t Read_MAX17048(uint8_t addressRegister)
{

  uint8_t D[2] = {0, 0};
  uint16_t valueDigital = 0;

  Wire1.beginTransmission(ADDRESS_MAX17048);
  Wire1.write(addressRegister);
  Wire1.endTransmission();

  Wire1.requestFrom(ADDRESS_MAX17048, 2);
  Wire1.readBytes(D, 2);

  valueDigital = (D[0] << 8) | D[1];

  return valueDigital;
}

void deviceUpdateStatus(void)
{
}

void statusInit(void)
{
  BaseType_t rc;
  LED_OFF;
  rc = xTaskCreatePinnedToCore(
      statusTask,
      "LED STATUS",
      STATUS_STACK_SIZE,
      nullptr,
      STATUS_TASK_PRIORITY,
      &statusTaskHandle,
      STATUS_TASK_CORE);

  assert(rc == pdPASS);
}

static void statusTask(void *pvParameters)
{
  static boolean _status = true;

  uint16_t _delay;

  for (;;)
  {
    _delay = device.delayStatus;
    if (_status)
    {
      LED_ON;
      if (device.connected)
      {
        if (device.sync)
        {
          if (device.active)
          {
            LED_OFF;
          }
          else
          {
            _delay >>= 2;
          }
        }
      }
      else
      {
        _delay >>= 2;
      }
      _status = false;
    }
    else
    {
      LED_OFF;
      if (device.connected)
      {
        if (device.sync)
        {
          if (!device.active)
          {
            _delay >>= 1;
          }
        }
      }
      else
      {
        _delay <<= 2;
      }
      _status = true;
    }
    vTaskDelay(pdMS_TO_TICKS(_delay));
  }
}