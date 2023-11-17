import serial
import pyautogui
import time

def scanformouseinput(ser):
    # Read a line of data (format: change_x,change_y,c_button,z_button)
    data = ser.readline().decode().strip()
    values = data.split(',')
    if len(values) != 4:
        return 0
    change_x, change_y, c_button, z_button = map(int, values)
    # Convert change_x and change_y to mouse movement
    pyautogui.move(change_x, change_y)
    # Simulate left and right clicks
    if c_button:
        pyautogui.click(button='left')
    if z_button:
        pyautogui.click(button='right')
    return 0

if __name__ == '__main__':
    print("Scanning for bluetooth mouse, press cntrl + c to exit")

    # Set the COM port to the one your Bluetooth device is connected to
    ser = serial.Serial('COM9', 9600)  # Replace 'COMx' with your COM port

    while(1):
        scanformouseinput(ser)
        # Wait for polling
        time.sleep(0.01)
