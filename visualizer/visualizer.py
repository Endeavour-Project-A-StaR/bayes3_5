import serial
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/rfcomm0'  # Change this to your Teensy's serial port!
BAUD_RATE = 115200

# Connect to the flight computer
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    print("Make sure the port is correct and the Arduino Serial Monitor is CLOSED.")
    exit()

# --- SETUP PLOT ---
fig = plt.figure(figsize=(12, 7))
fig.canvas.manager.set_window_title('Bayes 3.5 Ground Control Station')
plt.subplots_adjust(bottom=0.2) # Room for buttons

# --- Left Plot: 3D Rocket Attitude Indicator ---
ax_3d = fig.add_subplot(121, projection='3d')
ax_3d.set_xlim([-1.5, 1.5])
ax_3d.set_ylim([-1.5, 1.5])
ax_3d.set_zlim([-1.5, 1.5])
ax_3d.set_title("Rocket Attitude")

# Lock the camera to a nice isometric angle so you don't have to drag it
ax_3d.view_init(elev=20, azim=-45)

# Remove messy axis numbers for a cleaner look
ax_3d.set_xticks([])
ax_3d.set_yticks([])
ax_3d.set_zticks([])

# Create the 3 lines that make up the "Rocket"
# X: Nose to Tail (Red, thickest)
line_x, = ax_3d.plot([], [], [], color='#FF0000', linewidth=5, label='Fuselage (X)')
# Y: Fin Pair 1 (Green)
line_y, = ax_3d.plot([], [], [], color='#00FF00', linewidth=3, label='Fins (Y)')
# Z: Fin Pair 2 (Blue)
line_z, = ax_3d.plot([], [], [], color='#0000FF', linewidth=3, label='Fins (Z)')

# Add a distinct marker for the nose tip to show orientation clearly
nose_marker, = ax_3d.plot([], [], [], 'ro', markersize=8)

ax_3d.legend(loc='upper left')

# --- Right Plot: Servo Deflection Bar Chart ---
ax_bar = fig.add_subplot(122)
ax_bar.set_ylim([50, 130]) 
ax_bar.set_title("Live Servo Deflections")
ax_bar.set_ylabel("Degrees")

servo_labels = ['S1', 'S2', 'S3', 'S4']
bars = ax_bar.bar(servo_labels, [90, 90, 90, 90], color=['#FF595E', '#8AC926', '#1982C4', '#FFCA3A'])
ax_bar.axhline(90, color='black', linestyle='--', linewidth=1, label='Center (90°)')
ax_bar.legend()

# --- BUTTON CONTROLS ---
ax_btn_p = plt.axes([0.25, 0.05, 0.2, 0.075])
ax_btn_o = plt.axes([0.55, 0.05, 0.2, 0.075])

btn_p = Button(ax_btn_p, 'PREFLIGHT (P)', color='#d3d3d3', hovercolor='#a9a9a9')
btn_o = Button(ax_btn_o, 'OVERRIDE (O)', color='#ff9999', hovercolor='#ff6666')

def send_preflight(event):
    if ser and ser.is_open:
        ser.write(b'P')
        print("Command Sent: PREFLIGHT (P) - Alignment active, servos locked.")

def send_override(event):
    if ser and ser.is_open:
        ser.write(b'O')
        print("Command Sent: OVERRIDE (O) - Gyro integration active, servos LIVE.")

btn_p.on_clicked(send_preflight)
btn_o.on_clicked(send_override)

# --- BASE ROCKET GEOMETRY (Before Rotation) ---
# X axis goes from tail (-0.5) to nose (+1.5)
pts_x = np.array([[-0.5, 0, 0], [1.5, 0, 0]])
# Y fins go from -0.5 to +0.5 across the tail
pts_y = np.array([[0, -0.5, 0], [0, 0.5, 0]])
# Z fins go from -0.5 to +0.5 across the tail
pts_z = np.array([[0, 0, -0.5], [0, 0, 0.5]])

# --- ANIMATION LOOP ---
def update(frame):
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            
            if not line.startswith('{'):
                if line: print(f"FC MSG: {line}")
                return line_x, line_y, line_z, nose_marker, *bars
            
            data = json.loads(line)
            
            # --- 1. Update 3D Rocket Triad ---
            qw, qx, qy, qz = data['quats']
            rot = R.from_quat([qx, qy, qz, qw])
            
            # Rotate all the points mathematically
            rot_x = rot.apply(pts_x)
            rot_y = rot.apply(pts_y)
            rot_z = rot.apply(pts_z)
            
            # Apply to the Matplotlib lines
            line_x.set_data_3d(rot_x[:, 0], rot_x[:, 1], rot_x[:, 2])
            line_y.set_data_3d(rot_y[:, 0], rot_y[:, 1], rot_y[:, 2])
            line_z.set_data_3d(rot_z[:, 0], rot_z[:, 1], rot_z[:, 2])
            
            # Set the nose marker exactly at the tip of the X line
            nose_marker.set_data_3d([rot_x[1, 0]], [rot_x[1, 1]], [rot_x[1, 2]])
            
            # --- 2. Update Servo Bars ---
            servos = data['servo']
            for bar, val in zip(bars, servos):
                bar.set_height(val)
                if val <= 60.1 or val >= 119.9:
                    bar.set_color('red')
                else:
                    bar.set_color('#1982C4')
                
        except json.JSONDecodeError:
            pass 
        except KeyError:
            pass 
            
    return line_x, line_y, line_z, nose_marker, *bars

# Run the animation
ani = animation.FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)

plt.show()
