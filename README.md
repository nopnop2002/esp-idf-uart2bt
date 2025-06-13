# esp-idf-uart2bt
UART to Bluetooth bridge for ESP-IDF.

![Android-2](https://user-images.githubusercontent.com/6020549/173160564-4790a2cf-d084-400e-9a75-89cc2655f12e.JPG)

![urt2bt](https://user-images.githubusercontent.com/6020549/173161730-496501d3-163b-4e58-bb5b-f2fdfaa7854a.jpg)

# Software requirements
ESP-IDF V5.0 or later.   
ESP-IDF V4.4 release branch reached EOL in July 2024.   


# For classic bluetooth (ESP32 only)

```
git clone https://github.com/nopnop2002/esp-idf-uart2bt
cd esp-idf-uart2bt/classic_bt
idf.py set-target esp32
idf.py menuconfig
idf.py flash
```


# For ble 4.2 using ESP-Bluedroid host stack

```
git clone https://github.com/nopnop2002/esp-idf-uart2bt
cd esp-idf-uart2bt/bluedroid_ble
idf.py set-target {esp32/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

# For ble 4.2 using ESP-NimBLE host stack

```
git clone https://github.com/nopnop2002/esp-idf-uart2bt
cd esp-idf-uart2bt/nimble_ble
idf.py set-target {esp32/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

# Configuration
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

Strings from Arduino to ESP32 are terminated with CR(0x0d)+LF(0x0a).   
```
I (1189799) UART-RX: 0x3ffc8458   48 65 6c 6c 6f 20 57 6f  72 6c 64 20 31 30 30 31  |Hello World 1001|
I (1189799) UART-RX: 0x3ffc8468   0d 0a
```

The Arduino sketch inputs data with LF as the terminator.   
So strings from the ESP32 to the Arduino must be terminated with LF (0x0a).   
If the string output from the ESP32 to the Arduino is not terminated with LF (0x0a), the Arduino sketch will complete the input with a timeout.   
The default input timeout for Arduino sketches is 1000 milliseconds.   
The ESP32 SPP driver always sends data to the application with CR+LF as the termination character.   
This project changes the termination character from CR+LF to LF and sends the data to Arduino.   
The Arduino sketch will echo back the string it reads.   

```
I (1285439) UART-TX: 0x3ffc72f8   61 62 63 64 65 66 67 0a                           |abcdefg.|
I (1285459) UART-RX: 0x3ffc8458   61 62 63 64 65 66 67 0d  0a                       |abcdefg..|
```

# Connect ESP32 and AtMega328 using wire cable   

|AtMega328||ESP32|ESP32S3|ESP32C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|TX|--|GPIO16|GPIO2|GPIO1|5V to 3.3V level shifting is required|
|RX|--|GPIO17|GPIO1|GPIO0||
|GND|--|GND|GND|GND||

__You can change it to any pin using menuconfig.__   


# Android Application   
I used [this](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal) app.   

# iOS Application   
[This](https://apps.apple.com/jp/app/bluetooth-v2-1-spp-setup/id6449416841) might work, but I don't have iOS so I don't know.   

# References

https://github.com/nopnop2002/esp-idf-uart2web

https://github.com/nopnop2002/esp-idf-uart2udp

https://github.com/nopnop2002/esp-idf-uart2mqtt

https://github.com/nopnop2002/esp-idf-vcp2ble
