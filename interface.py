import serial

ser = serial.Serial(
    port='COM4',\
    baudrate=57600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=0
)

while True:
    for line in ser.read():
        print(line)

ser.close()