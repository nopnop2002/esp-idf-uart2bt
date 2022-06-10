# esp-idf-uart2bt
UART to Bluetooth bridge for ESP-IDF.

![urt2bt](https://user-images.githubusercontent.com/6020549/173161730-496501d3-163b-4e58-bb5b-f2fdfaa7854a.jpg)


# How to Install

- Write this sketch on Arduino Uno.   
You can use any AtMega328 microcontroller.   

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



- Configuration of esp-idf
```
idf.py set-target esp32
idf.py menuconfig
```

![config-top](https://user-images.githubusercontent.com/6020549/173160346-be330c9b-2aef-4d12-8906-9b3b3a4e0225.jpg)
![config-uart](https://user-images.githubusercontent.com/6020549/173160343-fcf36ffe-d51e-44bc-a299-8f3e2eb5fe9b.jpg)

- Connect ESP32 and AtMega328 using wire cable   

|AtMega328||ESP32|
|:-:|:-:|:-:|
|TX|--|GPIO16|
|RX|--|GPIO17|
|GND|--|GND|

__You can change it to any pin using menuconfig.__   


- Flash firmware
```
idf.py flash monitor
```

- Install Serial Bluetooth Terminal   
I used [this](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal) app.

- You can communicate to Arduino-UNO

![Android-1](https://user-images.githubusercontent.com/6020549/173160559-88c98af5-bb99-41ea-bd3d-1a7343fea5ad.JPG)
![Android-2](https://user-images.githubusercontent.com/6020549/173160564-4790a2cf-d084-400e-9a75-89cc2655f12e.JPG)

