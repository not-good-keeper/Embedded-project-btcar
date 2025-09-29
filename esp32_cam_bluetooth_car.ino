#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth with device name
  SerialBT.begin("ESP32-CAM-Car"); // Bluetooth device name for BT Car Controller app
  Serial.println("ESP32-CAM Bluetooth Ready!");
  Serial.println("Device Name: ESP32-CAM-Car");
  Serial.println("Waiting for Bluetooth connection...");
}

void loop() {
  // Handle Bluetooth commands and display in Serial Monitor
  if (SerialBT.available()) {
    String receivedData = SerialBT.readString();
    receivedData.trim(); // Remove any whitespace/newlines
    
    Serial.print("Received command: ");
    Serial.println(receivedData);
  }
  
  delay(20);
}
