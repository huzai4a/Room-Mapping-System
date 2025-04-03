import serial
import math
import numpy as np
import open3d as o3d
import time

# Initialize serial communication
with serial.Serial('COM5', 115200, timeout=2) as s:
    time.sleep(2)  # Wait for connection
    s.reset_input_buffer()  # Clear in buffer

    # Configuration
    x_increment = 500  # mm between scans
    x = 0
    all_points = []
    scan_point_counts = []  # Track points per scan

    with open("tof_radar_scans.xyz", "w") as f:
        for set_num in range(15):  # X scans
            f.write(f"# Set {set_num}\n")  # Identifier for each scan
            angle = 0
            points = []
            point_count = 0  # Count points in current scan

            while True:  # Keep waiting for valid data
                try:
                    line = s.readline().decode('ascii').strip()
                    length = len(line.split())
                    if not line or length != 32:
                        print("Waiting for valid data...")
                        time.sleep(0.1)  # Small delay before retrying
                        if length > 1:
                            print(f"Not enough data in the given set, need exactly 32 points for a full scan ({length} points received)")
                        continue  # Keep waiting instead of skipping

                    print(f"Received Set #{set_num+1}: {line}")

                    # Process dataset
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
                    break  # Exit while loop once valid data is processed

                except (ValueError, UnicodeDecodeError) as e:
                    print(f"Error: {e}")
                    time.sleep(0.1)  # Small delay before retrying

            all_points.extend(points)
            scan_point_counts.append(point_count)  # Store point count for this scan
            x += x_increment
            print(f"Scan {set_num+1} complete with {point_count} points")

# Debugging
print(f"Total points: {len(all_points)}")
print(scan_point_counts)
print(f"Avg points per scan: {np.mean(scan_point_counts)}")

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
