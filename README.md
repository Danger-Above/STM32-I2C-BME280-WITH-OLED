## STM32 I2C BME280 WITH OLED

This project is a small STM32 firmware, created to understand the I2C protocol and to 
learn to read and work with documentation of different peripherial devices: 
an environmental sensor (BME280) and an OLED display.  
Emphasis was also put on exploring more complex C programming concepts and basic embedded good practices.

**!!!Created for the NUCLEO-L476RG board!!!**

### Hardware used:
- STM32 NUCLEO-L476RG
- BME280 environmental sensor module (I2C)
- OLED display module (I2C, CH1116)
- USB connection to PC for UART logging (Virtual COM Port via ST-LINK)

### Features:
1. **I2C communication**  
   - I2C configured using STM32 HAL  
   - Verified using a logic analyzer
2. **BME280 sensor handling**  
   - Initialization and configuration  
   - Reading raw data and applying Bosch compensation formulas  
   - Temperature, pressure and humidity calculation
3. **OLED display handling**  
   - Minimal initialization sequence  
   - Text output on a single page using simple font
4. **UART logging**  
   - Used for initialization verification and debug output
   - Displays the same formatted data as shown on the OLED  

### Displayed data format:
```text
T:12.3 P:0456 H:78
``` 

### How to build and run:
1. **Prerequisites**
   - STM32CubeIDE (any L4-compatible version)
   - NUCLEO-L476RG board
   - USB cable
2. **Hardware connections**
   - I2C SCL -> PC0
   - I2C SDA -> PC1
3. **Building**
   - Open the project in STM32CubeIDE
   - Build
   - Flash to the board
4. **Running**
   - Connect USB to the ST-LINK USB port
   - Open terminal at 115200 8N1  

### Modules
- main (initialization, main loop)
- i2c_bus (common I2C abstraction layer)
- bme280 (sensor initialization and configuration, data readout, compensation)
- oled (display initialization and text rendering)
- logger (simple UART logging)  

### Artifacts
- Logic analyzer screenshots showing I2C transactions (ACK, reding registers, burst read)
- Photo of OLED displaying live sensor data

(Artifacts are stored in the `docs/` directory.)

### Known issues
- Data displayed on the OLED has a very simple format and is limited to one page of the display
- Text is sent to the display with a fixed buffer which results in zeros being sent if the text 
is smaller than the buffer
- I2C is implemented in blocking mode
- No I2C error handling, only one check with HAL_I2C_IsDeviceReady() is done