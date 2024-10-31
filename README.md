# Mqtt_esp32_IoT
Làm quen với esp32 kết nối wifi và sử dụng giao thức MQTT để truyền nhận dữ liệu đơn giản qua HIVEMQ MQTT broker
# How to do project:
Sử dụng framework Arduino của Platformio trên Visual Studio Code để lập trình Esp32 DevKit V1
  + Visual Studio Code
  + install extensions: Platformio
  + platform: espressif32
  + broad: esp32doit-devkit-v1
  + framework: arduino
  + add library:
    * DHT sensor library for ESPx
    * PubSubClient
  + build project -> upload project -> press and hold boot on esp32 to load code
# How to check data:
  + download and install: https://mqtt-explorer.com/
  + run app
  + add connections
  + name: IOT
  + protocol: mqtt://
  + host: 15dde8f446184978829a951a7530f634.s1.eu.hivemq.cloud
  + Port: 8883
  + username: iot2021
  + password: 12345678
  + encrytion(tls) ON
#good luck

    
