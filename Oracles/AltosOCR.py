from unittest import result
from PIL import Image
import easyocr
import mss
import pygetwindow as gw
import time
import re

class OCRScreenReader:
    def __init__(self):
        self.lat_deg = {"top": 360, "left": 1525, "width": 41, "height": 125//3}
        self.lat_min = {"top": 360, "left": 1574, "width": 100, "height": 125//3}

        self.long_deg = {"top": 400, "left": 1525, "width": 41, "height": 125//3}
        self.long_min = {"top": 400, "left": 1574, "width": 100, "height": 125//3}

        self.alt = {"top": 440, "left": 1522, "width": 120, "height": 125//3}
        winX, winY, winWidth, winHeight = (1920//2, 0, 1920//2, 1080)
        
        self.reader = easyocr.Reader(['en'])
        
        self.setup_window (winX, winY, winWidth, winHeight)
    def setup_window(self, x, y,width, height ):
        for window in gw.getAllWindows():
            
            if window.title.startswith("AltOS TeleBT"):
                window.resizeTo(width, height)
                window.moveTo(x, y)
            else:
                print("Window not found!")

    def parse_coordinates(self, text):
        text = text.replace("*", u"\u00B0").replace("\"",u"\u00B0").replace(",", ".")
        coord_pattern = re.compile(r"([NSWE]?)\s*(\d+)"+u"\u00B0"+"\s*(\d+)[\".]?(\d+)")

        # Search for latitude and longitude
        coords = coord_pattern.findall(text)
        formatted_coords = []
        for direction, degrees, minutes, seconds in coords:
            formatted = f"{degrees}.{str(float(minutes+'.'+seconds)/60)[2:]}"
            formatted_coords.append(formatted)

        if formatted_coords:
            return formatted_coords[0]
        else:
            return text
    

    def extend_image_with_white_background(self, image_path, output_path, extension_size=10):
        # Load the image
        img = Image.open(image_path)

        # Calculate the new size with extension
        new_width = img.width + 2 * extension_size
        new_height = img.height + 2 * extension_size

        # Create a new white background image
        background = Image.new('RGB', (new_width, new_height), color='white')

        # Calculate the position to paste the image (centered)
        paste_x = (new_width - img.width) // 2
        paste_y = (new_height - img.height) // 2

        # Paste the image onto the white background
        background.paste(img, (paste_x, paste_y))

        # Save the extended image
        background.save(output_path)

        return output_path

    def parse_mul_coordinates(self, lat_deg_results,lat_min_results, long_deg_results,long_min_results, alt_results):
        
        
        formatted_lat = f"{lat_deg_results[0][1]}.{str(float(lat_min_results[0][1])/60)[2:]}"
        formatted_long = f"-{long_deg_results[0][1]}.{str(float(long_min_results[0][1])/60)[2:]}"
        formatted_alt = f"{alt_results[0][1]}"
        if 0:
            print(f"Lat:{formatted_lat}")
            print(f"Long:{formatted_long}")
            print(f"Alt:{formatted_alt}")
        return (formatted_lat, formatted_long,formatted_alt)
        
    def mul_capture_screen_area(self):
        # Define screen area to capture
        al=u"\u00B0"+"0123456789m.\"'*"

        with mss.mss() as sct:
            lat_deg_ss = sct.grab(self.lat_deg)
            mss.tools.to_png(lat_deg_ss.rgb, lat_deg_ss.size, output='lat_deg.png')
            self.extend_image_with_white_background('lat_deg.png','lat_deg_ext.png',20)
            lat_deg_results = self.reader.readtext(image='lat_deg_ext.png',allowlist=al,min_size =4, text_threshold=.3 )
            
            lat_min_ss = sct.grab(self.lat_min)
            mss.tools.to_png(lat_min_ss.rgb, lat_min_ss.size, output='lat_min.png')
            lat_min_results = self.reader.readtext(image='lat_min.png', width_ths =100.0,allowlist=al)

            long_deg_ss = sct.grab(self.long_deg)
            mss.tools.to_png(long_deg_ss.rgb, long_deg_ss.size, output='long_deg.png')
            long_deg_results = self.reader.readtext(image='long_deg.png', width_ths =100.0,allowlist=al)
            
            long_min_ss = sct.grab(self.long_min)
            mss.tools.to_png(long_min_ss.rgb, long_min_ss.size, output='long_min.png')
            long_min_results = self.reader.readtext(image='long_min.png', width_ths =100.0,allowlist=al)
            
            alt_ss = sct.grab(self.alt)
            mss.tools.to_png(alt_ss.rgb, alt_ss.size, output='alt.png')
            alt_results = self.reader.readtext(image='alt.png', width_ths =100.0,allowlist=al)

            try:
                return self.parse_mul_coordinates(lat_deg_results,lat_min_results, long_deg_results,long_min_results, alt_results)
            except:
                
                print(f"EHH OHH\n{result}")
                return f"EHH OHH\n{result}"
    def capture_screen_area(self):
        # Define screen area to capture
        al=u"\u00B0"+"0123456789.\"'*"

        with mss.mss() as sct:
            screenshot = sct.grab(self.monitor)
            mss.tools.to_png(screenshot.rgb, screenshot.size, output='screen.png')
            results = self.reader.readtext(image='screen.png', width_ths =100.0,allowlist=al)
            try:
                self.parse_pos(results)
            except:
                print(f"EHH OHH\n{result}")
                

    def pos_parse(self, results):
        lat = self.parse_coordinates(results[0][1])
        long = self.parse_coordinates(results[1][1])
        alt = (float(results[2][1].replace(' ','').replace(',','.')))
        # print(f"Lat:{lat}")
        # print(f"Long:{long}")
        # print(f"Alt:{alt}")
        return (lat,long,alt)
if __name__ == "__main__":
    ocr_reader = OCRScreenReader()
    # ocr_reader.setup_window("AltusMetrum", 800, 600, 100, 100)
    while True:
        start_time = time.perf_counter()
        ocr_reader.mul_capture_screen_area()
        end_time = time.perf_counter() 
        print(f"OCR timing: {end_time-start_time:.6f} seconds")