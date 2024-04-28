# -*- coding: utf-8 -*-

import easyocr
import mss
import pygetwindow as gw
import time
import re

def setup_window(title, width, height, x, y):
    # Find the window by title
    window = gw.getWindowsWithTitle(title)[0]
    if window:
        # Move and resize window
        window.resizeTo(width, height)
        window.moveTo(x, y)
    else:
        print("Window not found!")
def parse_coordinates(text):
    text = text.replace("*", u"\u00B0").replace("\"",u"\u00B0").replace(",", ".")
    coord_pattern = re.compile(r"([NSWE]?)\s*(\d+)"+u"\u00B0"+"\s*(\d+)[\".]?(\d+)")

    # Search for latitude and longitude
    coords = coord_pattern.findall(text)
    formatted_coords = []
    for direction, degrees, minutes, seconds in coords:
        formatted = f"{degrees}\"{minutes}'{seconds}"
        formatted_coords.append(formatted)

    if formatted_coords:
        return formatted_coords[0]
    else:
        return text

def capture_screen_area(x, y, width, height):
    # Define screen area to capture
    monitor = {"top": y, "left": x, "width": width, "height": height}

    with mss.mss() as sct:
        screenshot = sct.grab(monitor)
        # Save screenshot to file
        mss.tools.to_png(screenshot.rgb, screenshot.size, output='screen.png')
        results = reader.readtext('screen.png')
        pos_parse(results)

def pos_parse(results):
    print(f"Lat:{parse_coordinates(results[0][1])}")
    print(f"Long:{parse_coordinates(results[1][1])}")
    print(f"Alt:{float(results[2][1].replace(' ','').replace(',','.'))}")
reader = easyocr.Reader(['en'])


#setup_window("AltusMetrum", 800, 600, 100, 100)
while 1:
    start_time = time.perf_counter()
    capture_screen_area(1933,600, 160, 110)
    end_time = time.perf_counter() 
    print(f"OCR timing: {end_time-start_time:.6f} seconds")