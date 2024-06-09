from pynput import mouse
import pygetwindow as gw

def get_window_at_position(x, y):
    """Get the title of the window at the specified screen coordinates."""
    for window in gw.getAllWindows():
        if window.left <= x <= window.left + window.width and window.top <= y <= window.top + window.height:
            return window.title
    return "No window found at this position"

def on_move(x, y):
    print(f"Mouse moved to ({x}, {y})")

def on_click(x, y, button, pressed):
    if pressed:
        window_title = get_window_at_position(x, y)
        print(f"Mouse clicked at ({x}, {y}) on window titled: '{window_title}'")

# Set up mouse listener
with mouse.Listener(on_move=on_move, on_click=on_click) as listener:
    listener.join()