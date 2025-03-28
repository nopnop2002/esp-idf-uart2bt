# For ble 4.2 using ESP-Bluedroid host stack
ESP-IDF can use either the ESP-Bluedroid host stack or the ESP-NimBLE host stack.   
The differences between the two are detailed [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/ble/overview.html).   
This project uses the ESP-Bluedroid host stack.   

- pair with ESP_SPP_SERVER   

- Launch the app and select device  
Menu->Devices->Bluetooth LE   

- Long press the device and select the Edit menu   
![ble-1](https://github.com/user-attachments/assets/12f2f875-59dc-474d-9aae-711bea2586bb)

- Select Custom and specify UUID   
![ble-2](https://user-images.githubusercontent.com/6020549/184459827-f62dc206-6bc2-41a7-9a88-74b9c84bbb89.JPG)

- Connect to device   
You can communicate to UNO using android.   
![ble-3](https://github.com/user-attachments/assets/b8453c01-e217-43dd-9839-a6750038dbac)
