import serial
import time

# Replace 'COM3' with the port your Arduino is connected to.
# On Unix-like systems, it might look like '/dev/ttyUSB0' or '/dev/ttyACM0'
ser = serial.Serial('COM6', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

def send_data(lat, lon, alt):
    data = "{},{},{}\n".format(lat, lon, alt)
    ser.write(data.encode())

# Define a list of test data points (latitude, longitude, altitude)
test_data = [
    (44.56777, 123.27502, 100),
    (44.56849, 123.275, 300),
    (44.56900, 123.27600, 150),
    (44.56950, 123.27700, 200),
    # Add as many test data points as needed
]

try:
    while 1:
        for lat, lon, alt in test_data:
            send_data(lat, lon, alt)
            print(f"Sent data: {lat}, {lon}, {alt}")
            time.sleep(1)  # Adjust as needed for your testing speed
        time.sleep(3)
except KeyboardInterrupt:
    print("Program stopped")

ser.close()  # Close the serial connection