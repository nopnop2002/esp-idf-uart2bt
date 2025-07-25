# For ble 4.2 using ESP-NimBLE host stack
ESP-IDF can use either the ESP-Bluedroid host stack or the ESP-NimBLE host stack.   
The differences between the two are detailed [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/ble/overview.html).   
This project uses the ESP-NimBLE host stack.   

- pair with ESP_NIMBLE_SERVER   

- Launch the app and select device  
Menu->Devices->Bluetooth LE   

- Long press the device and select the Edit menu   
![Image](https://github.com/user-attachments/assets/2d36b757-585a-4310-919c-a57f136c7f20)

- Select Custom and specify UUID   
The UUIDs are different for ESP-Bluedroid and ESP-NimBLE.   
![Image](https://github.com/user-attachments/assets/9b0f23bc-86f4-4631-81e6-1df8d876f41b)

- Connect to device   
You can communicate to UNO using android.   
![Image](https://github.com/user-attachments/assets/e84fa3b1-a0ee-4af3-a64c-695a5b383857)


# Concurrent connection
Unlike ESP-Bluedroid host stack, ESP-NimBLE host stack allows concurrent connections.   
The maximum number of simultaneous connections is specified here.   
![Image](https://github.com/user-attachments/assets/9d1e1182-ed41-4b9e-bc55-bb3c75dd4745)   
![Image](https://github.com/user-attachments/assets/4d84823a-69c4-48bf-9671-64644f048ccd)   
