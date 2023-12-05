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
    #print(values)
    change_x, change_y, z_button, c_button = map(int, values)

    if(change_x > 130):
        change_x = change_x- 130
    elif(change_x < 100):
        change_x = -(100 - change_x)
    else:
        change_x = 0

    if(change_y > 130):
        change_y = -(change_y - 130)
    elif(change_y < 100):
        change_y = 100 - change_y
    else:
        change_y = 0


    # Convert change_x and change_y to mouse movement
    pyautogui.move(change_x, change_y)

    # Simulate left and right clicks
    if not c_button:
        pyautogui.click(button='left')
    if not z_button:
        pyautogui.click(button='right')
