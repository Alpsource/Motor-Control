import serial
from time import sleep

ser = serial.Serial ("COM13", 115200)    #Open port with baud rate
while True:
    _data = input("Enter String Data: ")
    ser.write(bytes(_data,'utf-8'))