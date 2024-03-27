# esp-idf-uart2bt
UART to Bluetooth bridge for ESP-IDF.

![Android-2](https://user-images.githubusercontent.com/6020549/173160564-4790a2cf-d084-400e-9a75-89cc2655f12e.JPG)

![urt2bt](https://user-images.githubusercontent.com/6020549/173161730-496501d3-163b-4e58-bb5b-f2fdfaa7854a.jpg)

# Software requirements
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   


# Using classic bluetooth (ESP32 only)

```
git clone https://github.com/nopnop2002/esp-idf-uart2bt
cd esp-idf-uart2bt/classic_bt
idf.py set-target esp32
idf.py menuconfig
idf.py flash
```


# Using ble 4.2

```
git clone https://github.com/nopnop2002/esp-idf-uart2bt
cd esp-idf-uart2bt/ble
idf.py set-target {esp32/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

![config-top](https://user-images.githubusercontent.com/6020549/173160346-be330c9b-2aef-4d12-8906-9b3b3a4e0225.jpg)
![config-uart](https://user-images.githubusercontent.com/6020549/173160343-fcf36ffe-d51e-44bc-a299-8f3e2eb5fe9b.jpg)



# Write this sketch on Arduino Uno.   
You can use any AtMega microcontroller.   

```
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Serial.println(command);
  }

  if(lastMillis + 1000 <= millis()){
    Serial.print("Hello World ");
    Serial.println(millis());
    lastMillis += 1000;
  }

  delay(1);
}
```


# Connect ESP32 and AtMega328 using wire cable   

|AtMega328||ESP32|ESP32S3|ESP32C2/C3/C6|
|:-:|:-:|:-:|:-:|:-:|
|TX|--|GPIO16|GPIO2|GPIO1|
|RX|--|GPIO17|GPIO1|GPIO0|
|GND|--|GND|GND|GND|

__You can change it to any pin using menuconfig.__   


# Install Serial Bluetooth Terminal   
I used [this](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal) app.   
