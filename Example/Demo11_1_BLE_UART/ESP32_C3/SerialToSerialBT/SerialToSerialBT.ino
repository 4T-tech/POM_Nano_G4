#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

String device_name = "ESP32C6-BLE-UART";

// BLE服务UUID（标准串口服务）
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
// 特征UUID（用于接收数据，手机→ESP32）
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
// 特征UUID（用于发送数据，ESP32→手机）
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// 回调类：处理连接状态
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      pServer->startAdvertising(); // 重新开始广播
    }
};

// 回调类：处理接收数据
class MyCallbacks: public BLECharacteristicCallbacks {
private:
    String lastValue = "";
    unsigned long lastTime = 0;
    
    void onWrite(BLECharacteristic *pCharacteristic) {
      String currentValue = pCharacteristic->getValue().c_str();
      if(millis() != lastTime || (currentValue != lastValue && currentValue.length() > 0))
      {
        lastTime = millis();
        lastValue = currentValue;

        Serial.write((const uint8_t*)currentValue.c_str(), currentValue.length());
      }
    }
};

void setup() {
  Serial.begin(460800);
  
  // 初始化BLE设备
  BLEDevice::init(device_name.c_str());
  
  // 创建BLE服务器
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 创建BLE服务
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 创建用于发送的特征
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // 创建用于接收的特征
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // 启动服务
  pService->start();

  // 开始广播
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // 有助于iPhone连接
  BLEDevice::startAdvertising();
  
  Serial.println("BLE UART Service Started");
  Serial.print("Device Name: ");
  Serial.println(device_name);
  Serial.println("Waiting for client connection...");
}

void loop() {
  // 处理连接状态变化
  if (!deviceConnected && oldDeviceConnected) {
    delay(200); // 给蓝牙栈一点时间
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // 从串口读取数据并通过BLE发送
  if (deviceConnected && Serial.available()) {
    String input = Serial.readStringUntil('\n');
    pTxCharacteristic->setValue((uint8_t*)input.c_str(), input.length());
    pTxCharacteristic->notify();
  }
}