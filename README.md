# BluetoothNunchuckMouse
STM32 Implementation of Nunchuck-Controlled Bluetooth Mouse. Designed for L053R8 but will work for most boards with common peripherals.

To build (after cloning project into top-level STM32 project directory):

1. Create a new STM project with the following peripherals enabled:
- UART (for bluetooth)
- I2C (for nunchuck)
- SPI (optional) (If you wish to use an LCD display)

2. Copy the STM/Drivers/LCD and STM/Drivers/UGUI into Drivers folder of top-level STM project. Add to include path.

3. If you are not using X11 forwarding, comment out the X11 files in STM/Drivers/UGUI.

4. Copy the STM/Src/main.c folder into the Core/Src/ folder in your STM project (Or wherever your main program is housed). You will likely have to change some of the HAL pins from what we have to whatever board you are using.

5. Program board.

6. Connect host PC to bluetooth module.

7. Install packages and run Python code:
~~~
pip3 install serial
pip3 install pyautogui
python3 ./Python/main.py
~~~
