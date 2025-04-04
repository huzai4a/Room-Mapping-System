import serial
ser = serial.Serial('COM5', 115200, timeout=1)  # Replace COMX with your port
ser.close()
ser.open()
