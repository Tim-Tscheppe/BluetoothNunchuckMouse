# BluetoothNunchuckMouse
STM32 Implementation of Nunchuck-Controlled Bluetooth Mouse. Designed for L053R8 but will work for most boards with common peripherals.

To build (after cloning project into top-level STM32 project directory):

1. Create a new STM project with the following peripherals enabled:
- UART (for bluetooth)
- I2C (for nunchuck)

2. Copy the STM/Src/main.c folder into the Core/Src/ folder in your STM project (Or wherever your main program is housed). You will likely have to change some of the HAL pins from what we have to whatever board you are using.

3. Program board.

4. Connect host PC to bluetooth module.

5. Install packages and run Python code:
~~~
pip3 install serial
pip3 install pyautogui
python3 ./Python/main.py
~~~
