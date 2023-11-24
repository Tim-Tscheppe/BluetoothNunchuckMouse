import serial
import pyautogui

# Modify as needed
port = 'COM5'
baudrate = 9600

# Connect to serial, may need to loop this to try multiple times
try:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.SEVENBITS
    )
    print('Connected successfully to ' + port)
except:
    print('Could not connect to ' + port)
    exit(1)

print('Nunchuck remotely controlling mouse, press ctrl + c to exit')

# Infinite loop to update mouse movements
while True:

    line = ser.readline()
    x, y, z, c = map(int, line.split(','))

    # TODO: Figure out how x and y map to the screen (may need to comment this out initially or scale nunchuck values)
    pyautogui.moveTo(x, y, duration=0.1)
    if(z == 1):
        pyautogui.click()
    if(c == 1):
        pyautogui.rightClick()
