"""

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import open3d as o3d
import numpy as np


xyz_points = np.array([
    [0, 0, 0],
    [10.606, 10.606, 2.598],
    [10.000, 17.321, 5.177]
])

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_points)
o3d.visualization.draw_geometries([pcd])



s = serial.Serial('COM5', 115200, timeout = 5)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()






# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())

# recieve 10 measurements from UART of MCU 
for i in range(20):
    x = s.readline()

    try:
        print(int (x.decode()), end=" ")
        print("integer")
    except:
        print((x.decode()))
       
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()
"""




import serial
import math
import numpy as np
import open3d as o3d
import time

# Initialize serial communication
with serial.Serial('COM5', 115200, timeout=2) as s:
    time.sleep(2)  # Wait for connection
    s.reset_input_buffer()

    print("Waiting for data...")

    # Configuration
    x_increment = 500 #cm between scans
    x = 0
    all_points = []
    scan_point_counts = []  # Track points per scan

    for set_num in range(7):  # X scans
        with open(f"tof_radar_set_{set_num}.xyz", "w") as f:
            angle = 0
            points = []
            point_count = 0  # Count points in current scan

            for _ in range(32):  # 32 measurements per scan
                try:
                    line = s.readline().decode('ascii').strip()
                    print(f"Received: {line}")

                    if not line or len(line.split()) != 32:
                        continue

                    distances = list(map(float, line.split()))
                    for j, dist in enumerate(distances):
                        theta = math.radians(angle + (11.25 * j))
                        y = dist * math.cos(theta)
                        z = dist * math.sin(theta)
                        f.write(f"{x} {y} {z}\n")
                        points.append([x, y, z])
                        point_count += 1

                    angle += 11.25
                    f.flush()

                except (ValueError, UnicodeDecodeError) as e:
                    print(f"Error: {e}")
                    continue

            all_points.extend(points)
            scan_point_counts.append(point_count)  # Store point count for this scan
            x += x_increment
            print(f"Scan {set_num+1} complete with {point_count} points")

# Create line connections
lines = []
point_offset = 0

for scan_idx in range(len(scan_point_counts)):
    num_points = scan_point_counts[scan_idx]
    
    # Connect points within scan (ring)
    for i in range(num_points - 1):
        lines.append([point_offset + i, point_offset + i + 1])
    lines.append([point_offset + num_points - 1, point_offset])  # Close ring
    
    # Connect to next scan (if exists)
    if scan_idx < len(scan_point_counts) - 1:
        next_scan_offset = point_offset + num_points
        next_scan_points = scan_point_counts[scan_idx + 1]
        
        # Connect corresponding points (simplified - assumes equal point counts)
        for i in range(min(num_points, next_scan_points)):
            lines.append([point_offset + i, next_scan_offset + i])
    
    point_offset += num_points

# Create combined visualization
combined_pcd = o3d.geometry.PointCloud()
combined_pcd.points = o3d.utility.Vector3dVector(np.array(all_points))

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(all_points),
    lines=o3d.utility.Vector2iVector(lines)
)

# Custom visualization
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(combined_pcd)
vis.add_geometry(line_set)

# Set better viewing angle
ctr = vis.get_view_control()
ctr.set_front([0, -1, 0])  # Looking along the depth axis
ctr.set_up([0, 0, 1])  # Z-axis is up

print("Showing connected 3D structure")
vis.run()
