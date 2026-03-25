import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time



# Serial Port Configuration
COM_PORT = 'COM4'   # Adjust accodringly
BAUDRATE = 115200
PACKET_SIZE = 20    # 4 floats + 1 uint32_t = 20 bytes

# Plot Configuration
WINDOW_SIZE = 200

CHANNELS = [0, 3]
CHANNEL_NAMES = ['Ia', 'Speed(RPM)']

Y_LIMITS = [(-10, 10), (0, 4000)]

# Initialize data queues for each channel
data_queues = [deque(maxlen=WINDOW_SIZE) for _ in CHANNELS]

# Serial Connection
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)

# Packet Parsing Function
def parse_packet(data: bytes):
    try:
        ia, ib, ic, speed, pos = struct.unpack('<ffffI', data)
        return ia, ib, ic, speed, pos
    except struct.error:
        return None

def read_serial():
    buffer = b''          # Buffer to hold incoming data
    while True:
        # Read available data from serial port
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buffer += chunk

        # Process complete packets in the buffer
        while len(buffer) >= PACKET_SIZE:
            # Extract one packet
            packet = buffer[:PACKET_SIZE]
            buffer = buffer[PACKET_SIZE:]

            # Parse the packet
            values = parse_packet(packet)
            if values is not None:
                # Add the required channel data to the queue
                for i, ch in enumerate(CHANNELS):
                    data_queues[i].append(values[ch])
            else:
                # Parsing error, print raw data for debugging
                print("Packet parse error, data:", packet.hex())

# Start the serial reading thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# ================== Plotting ==================
fig, axes = plt.subplots(len(CHANNELS), 1, sharex=True)
if len(CHANNELS) == 1:
    axes = [axes]   # Ensure axes is always a list for consistent indexing

lines = []
for i, ax in enumerate(axes):
    line, = ax.plot([], [], label=CHANNEL_NAMES[i])
    lines.append(line)
    ax.set_ylim(*Y_LIMITS[i])
    ax.set_ylabel(CHANNEL_NAMES[i])
    ax.grid(True)
    ax.legend(loc='upper right')
axes[-1].set_xlabel('Sample')

def animate(frame):
    for i, queue in enumerate(data_queues):
        if len(queue) > 0:
            ydata = list(queue)
            xdata = list(range(len(ydata)))
            lines[i].set_data(xdata, ydata)
            # Set x-axis limits to show the latest data
            axes[i].set_xlim(0, max(len(ydata), WINDOW_SIZE))
    return lines

# Start the animation with a short interval for smoother updates
ani = animation.FuncAnimation(fig, animate, interval=50, blit=False)

plt.tight_layout()
plt.show()

# Release the serial port when the program is closed
ser.close()