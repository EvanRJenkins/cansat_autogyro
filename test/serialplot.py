import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import random
import math

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'
BAUD_RATE = 115200       # Match this to your Arduino code
MAX_POINTS = 200       # How many points to show on the graph at once

# --- DATA STORAGE ---
data_buffer = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_lock = threading.Lock()
is_running = True

def get_serial_data():
    """
    Thread that runs in the background to read data from the serial port.
    It separates data reading from UI updating to prevent freezing.
    """
    print(f"Connecting to {PORT} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Allow connection to settle (crucial for Arduino)
        print("Connected!")
        
        while is_running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                   
                    # Ignore empty lines
                    if not line:
                        continue
                    value = float(line)
                else:
                    continue # No data waiting
                
                # specific safe update of the data buffer
                with data_lock:
                    data_buffer.append(value)
                    
            except ValueError:
                pass # Handled corrupted data (common on serial startup)
            except Exception as e:
                print(f"Error reading data: {e}")
                
    except serial.SerialException:
        print(f"Could not open port {PORT}.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

def update_plot(frame, line, ax):
    """
    Called periodically by Matplotlib to update the graph.
    """
    with data_lock:
        # Copy data to ensure thread safety during plotting
        accel_data_x = list(data_buffer)
        
    line.set_ydata(accel_data_x)
    # Y axis limits
    ax.set_ylim(-1000, 1000)
    
    return line,

def main():
    global is_running
    
    # 1. Start the serial reading thread
    serial_thread = threading.Thread(target=get_serial_data)
    serial_thread.daemon = True # Ensures thread dies when main program exits
    serial_thread.start()

    # 2. Setup the Plot
    fig, ax = plt.subplots()
    ax.set_title("Live Serial Data")
    ax.set_xlabel("Time (last 100 points)")
    ax.set_ylabel("Sensor Value")
    
    # Initialize x-axis (fixed range for rolling window)
    x_data = range(MAX_POINTS)
    ax.set_xlim(0, MAX_POINTS - 1)
    
    # Create the line object
    line, = ax.plot(x_data, [0]*MAX_POINTS, lw=2)

    # 3. Start Animation
    # interval=50 means update every 50ms (20 FPS)
    ani = animation.FuncAnimation(fig, update_plot, fargs=(line, ax), interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        serial_thread.join(timeout=1)
        print("Exiting...")

if __name__ == "__main__":
    main()
