import serial
import os
import time

# from WAVESHARE, establish serial connection.
def connect_serial(port, baudrate):
    while True:
        try:
            ser = serial.Serial(port, baudrate)
            print("Serial is connected: {}".format(ser.is_open))
            return ser
        except serial.SerialException as e:
            print("Serial Disconnected:", e)
            print("wait for 5 second for reconnecting...")
            time.sleep(5)
##kill the agetty serial
os.system("sudo systemctl stop serial-getty@ttyS0.service")
os.system("sudo chmod 777 /dev/ttyS0")
port = "/dev/ttyS0"
baudrate = 115200
ser = connect_serial(port, baudrate)

while True:
    try:
        value_str = ser.readline().decode().strip()
        print("value_str", value_str)
    except KeyboardInterrupt:
        ser.close()
        