import serial
import pyautogui

# Set the COM port to the one your Bluetooth device is connected to
ser = serial.Serial('COM9', 9600)  # Replace 'COMx' with your COM port
print("Connected to " + ser.portstr)
while True:
    # Read a line of data (format: change_x,change_y,c_button,z_button)
    try:
        data = ser.readline().decode().strip()
        values = data.split(',')
    except:
        #will get triggered if the BT connection is made while data is in the middle of being sent
        print("Invalid data: " + data)
        continue

    if len(values) != 4:
        print("Invalid data: " + data)
        continue
    print(values)
    change_x, change_y, c_button, z_button = map(int, values)

    # Convert change_x and change_y to mouse movement
    pyautogui.move(change_x, change_y)

    # Simulate left and right clicks
    if c_button:
        pyautogui.click(button='left')
    if z_button:
        pyautogui.click(button='right')
