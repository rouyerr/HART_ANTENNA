import serial
import time
import threading

ser = serial.Serial('COM5', 9600, timeout=1)

# Global variables to hold the latest position data and a lock for synchronization
latest_position = (0,0,0)
position_lock = threading.Lock()

def send_data(lat, lon, alt):
    data = "{},{},{}\n".format(lat, lon, alt)
    ser.write(data.encode())
      

def handle_serial_message(decoded_byte):
    global latest_position
    global position_lock

    if decoded_byte == "\x05":
        print(f"Got request for data")
        position_input = input("Enter latitude, longitude, and altitude (comma-separated): ")
        try:
            lat, lon, alt = map(float, position_input.split(','))
            
            with position_lock:
                latest_position = (lat, lon, alt)
            send_data(lat, lon, alt)
            print(f"Sent data: Lat={lat}, Lon={lon}, Alt={alt}")
        except ValueError:
            print("Invalid input format. Please enter latitude, longitude, and altitude in decimal format separated by commas.")
            # Send dummy data to maintain communication with the other program
            with position_lock:
                lat, lon, alt = latest_position
            send_data(lat, lon, alt)
            print(f"Sent dummy data: Lat={lat}, Lon={lon}, Alt={alt}")
    else:
        message = ser.read_until(b'\n').decode().strip()
        print(f"Display message: {decoded_byte}{message}")
def serial_thread():
    
    try:
        
        while True:
            # Read a byte from the serial port
            byte = ser.read(1)

            # Decode the byte
            decoded_byte = byte.decode()

            # Handle the message based on its type
            handle_serial_message(decoded_byte)
    except KeyboardInterrupt:
        print("Serial thread stopped")


# Start serial communication loop
        
serial_thread()

ser.close()  # Close the serial connection
