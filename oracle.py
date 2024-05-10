import serial
import time
import threading
from AltosOCR import OCRScreenReader 

ser = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)
ocr = OCRScreenReader()

# Global variables to hold the latest position data and a lock for synchronization
latest_position = (0,0,0)
position_lock = threading.Lock()

def send_data(lat, lon, alt):
    data = "{},{},{}\n".format(lat, lon, alt)
    ser.write(data.encode())

def ocr_thread():
    global latest_position
    global position_lock
    try:
        while True:
            start_time = time.time()
            # OCR processing
            try:
                lat, lon, alt = ocr.mul_capture_screen_area()  # Update with actual OCR method

                # Acquire the lock before updating the shared data
                with position_lock:
                    latest_position = (lat, lon, alt)
                end_time = time.time()
                print(f"Updated position: Lat={lat}, Lon={lon}, Alt={alt}, Latency = {end_time - start_time} s")
            except Exception as e:
                print(f"Error occurred: {e}")
    except KeyboardInterrupt:
        print("OCR thread stopped")
        

def handle_serial_message(decoded_byte):
    global latest_position
    global position_lock

    if decoded_byte == "\x05":
        print(f"Got request for data")
        # Signal to send the latest position data
        with position_lock:
            lat, lon, alt = latest_position
        send_data(lat, lon, alt)
        print(f"Sent data: Lat={lat}, Lon={lon}, Alt={alt}")
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

# Create and start OCR thread
ocr_thread = threading.Thread(target=ocr_thread)
ocr_thread.daemon = True
ocr_thread.start()

# Start serial communication loop
serial_thread()

ser.close()  # Close the serial connection
