import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'
BAUD_RATE = 230400
MAX_POINTS = 500
NUM_TRACES = 3        # Set this to match the number of values you send (e.g., 3 for X,Y,Z)
Y_AXIS_MIN = -1100    # Initial min (will auto-scale if auto_scale is True)
Y_AXIS_MAX = 1100     # Initial max

# --- DATA STORAGE ---
# Create a list of deques, one for each trace
data_buffers = [deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS) for _ in range(NUM_TRACES)]
data_lock = threading.Lock()
is_running = True

def get_serial_data():
    print(f"Connecting to {PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Connected!")
        
        while is_running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if not line:
                        continue
                    
                    # Expect comma-separated values: "100, 200, 300"
                    parts = line.split(',')
                    
                    # Only process if we get exactly the expected number of values
                    if len(parts) == NUM_TRACES:
                        values = [float(p) for p in parts]
                        
                        with data_lock:
                            for i in range(NUM_TRACES):
                                data_buffers[i].append(values[i])
                else:
                    time.sleep(0.01) # Slight release of CPU
                    
            except ValueError:
                pass # Ignore corrupt lines
            except Exception as e:
                print(f"Error reading data: {e}")
                
    except serial.SerialException:
        print(f"Could not open port {PORT}.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

def update_plot(frame, lines, ax):
    with data_lock:
        # Create a snapshot of data for all traces
        current_data = [list(buf) for buf in data_buffers]

    # Update each line on the graph
    for i, line in enumerate(lines):
        line.set_ydata(current_data[i])

    return lines

def main():
    global is_running
    
    # --- AESTHETIC SETUP ---
    # 1. Enable Dark Mode
    plt.style.use('dark_background')
    
    # 2. Define Neon Colors (Cyan, Lime, Magenta, Yellow)
    neon_colors = ['#00FFFF', '#39FF14', '#FF00FF', '#FFFF00']

    # --- THREAD SETUP ---
    serial_thread = threading.Thread(target=get_serial_data)
    serial_thread.daemon = True
    serial_thread.start()

    # --- PLOT SETUP ---
    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title('Neon Serial Plotter') # Window Title

    # 3. Toggle Full Screen
    # Note: This command works on most backends (Windows/Linux/Mac)
    # If it fails, try: fig.canvas.manager.window.state('zoomed')
    try:
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
    except Exception:
        pass

    ax.set_title(f"LIVE DATA STREAM // {NUM_TRACES} INPUTS", fontsize=16, fontweight='bold', color='white')
    ax.set_xlabel("TIME STEPS", fontsize=10)
    ax.set_ylabel("AMPLITUDE", fontsize=10)
    
    # Customize the Grid (Faint and dashed)
    ax.grid(True, color='gray', linestyle='--', linewidth=0.5, alpha=0.3)
    
    # Remove top and right spines for a cleaner look
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Setup X-axis
    x_data = range(MAX_POINTS)
    ax.set_xlim(0, MAX_POINTS - 1)
    ax.set_ylim(Y_AXIS_MIN, Y_AXIS_MAX)

    # Create Lines with Neon Colors
    lines = []
    labels = ["ACCEL_X", "ACCEL_Y", "ACCEL_Z"]
    
    for i in range(NUM_TRACES):
        label_text = labels[i] if i < len(labels) else f"TRACE_{i}"
        
        # Cycle through neon colors using modulo operator
        color = neon_colors[i % len(neon_colors)]
        
        # lw=2 makes the line thicker (more neon-like)
        line, = ax.plot(x_data, [0]*MAX_POINTS, label=label_text, lw=2, color=color)
        lines.append(line)
    
    # Legend with dark background
    legend = ax.legend(loc="upper left", facecolor='black', framealpha=1)
    for text in legend.get_texts():
        text.set_color("white")

    # Start Animation
    ani = animation.FuncAnimation(fig, update_plot, fargs=(lines, ax), interval=50, blit=False)

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
