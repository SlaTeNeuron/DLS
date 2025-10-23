#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

# Read the speed data
print('Loading speed data from OptimizedSpeeds.csv...')
data = pd.read_csv('OutputData/OptimizedSpeeds.csv')

# Create the plots - 2x2 grid layout
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

# Plot 1: Racing line colored by velocity
x = data['X'].values
y = data['Y'].values
velocities_mph = data['Velocity_mph'].values

# Track limit data
track_x = data['trackX'].values
track_y = data['trackY'].values
track_tangent_direction = data['trackTangentDirection'].values

# Filter data to only show points within ±30m on x-axis
x_limit = 30.0
mask = (x >= -x_limit) & (x <= x_limit)
x_filtered = x[mask]
y_filtered = y[mask]
velocities_filtered = velocities_mph[mask]

# Filter track data using the same mask
track_x_filtered = track_x[mask]
track_y_filtered = track_y[mask]
track_tangent_filtered = track_tangent_direction[mask]

print(f'Filtering track data: showing {len(x_filtered)} of {len(x)} points within ±{x_limit}m')

# Calculate track limits (±150cm = ±1.5m from track centerline)
track_width_half = 1.5  # 150cm in meters

# Calculate perpendicular direction (tangent + pi/2)
perp_direction = -track_tangent_filtered

# Calculate track limit coordinates
left_limit_x = track_x_filtered + track_width_half * np.cos(perp_direction)
left_limit_y = track_y_filtered + track_width_half * np.sin(perp_direction)
right_limit_x = track_x_filtered - track_width_half * np.cos(perp_direction)
right_limit_y = track_y_filtered - track_width_half * np.sin(perp_direction)

# Create color map from blue (slow) to red (fast)
colors = ['#0000FF', '#00FFFF', '#00FF00', '#FFFF00', '#FF8000', '#FF0000']
n_bins = 100
cmap = LinearSegmentedColormap.from_list('speed', colors, N=n_bins)

# Create the colored line plot using filtered data
points = np.array([x_filtered, y_filtered]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

from matplotlib.collections import LineCollection
lc = LineCollection(segments, cmap=cmap, linewidths=2)
lc.set_array(velocities_filtered[:-1])  # Use velocity for coloring
line = ax1.add_collection(lc)

# Plot track limits
ax1.plot(left_limit_x, left_limit_y, 'k--', linewidth=0.5, alpha=0.7, label='Track Limits')
ax1.plot(right_limit_x, right_limit_y, 'k--', linewidth=0.5, alpha=0.7)

# Set plot properties with fixed x-axis limits
ax1.set_xlim(-x_limit, x_limit)
ax1.set_ylim(y_filtered.min() - 5, y_filtered.max() + 5)
ax1.set_aspect('equal', adjustable='box')
ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.set_title(f'Racing Line with Track Limits (±{x_limit}m view)\n(Blue=Slow, Red=Fast, Track Width=3m)')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper right')

# Add colorbar
cbar = fig.colorbar(line, ax=ax1, shrink=0.8)
cbar.set_label('Speed (mph)', rotation=270, labelpad=20)

# Plot 2: Speed vs time
time = data['Time_s'].values
ax2.plot(time, velocities_mph, 'b-', linewidth=2, label='Optimized Speed')
ax2.fill_between(time, velocities_mph, alpha=0.3, color='lightblue')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Speed (mph)')
ax2.set_title('Speed Profile vs Time')
ax2.grid(True, alpha=0.3)
ax2.legend()

# Add statistics text
min_speed = velocities_mph.min()
max_speed = velocities_mph.max()
avg_speed = velocities_mph.mean()
stats_text = f'Min: {min_speed:.1f} mph\nMax: {max_speed:.1f} mph\nAvg: {avg_speed:.1f} mph'
ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

# Plot 3: G-Forces vs Time
lateral_g = data['LateralG'].values
longitudinal_g = data['LongitudinalG'].values
total_g = data['TotalG'].values

ax3.plot(time, lateral_g, 'r-', linewidth=2, label='Lateral G', alpha=0.8)
ax3.plot(time, longitudinal_g, 'b-', linewidth=2, label='Longitudinal G', alpha=0.8)
ax3.plot(time, total_g, 'k--', linewidth=0.5, label='Total G', alpha=0.6)
ax3.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('G-Force')
ax3.set_title('G-Forces vs Time')
ax3.grid(True, alpha=0.3)
ax3.legend()

# Plot 4: Curvature vs Time
curvature = data['Curvature'].values
ax4.plot(time, curvature, 'g-', linewidth=2, label='Track Curvature', alpha=0.8)
ax4.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
ax4.fill_between(time, curvature, alpha=0.3, color='lightgreen')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Curvature (1/m)')
ax4.set_title('Track Curvature vs Time\n(Positive=Right Turn, Negative=Left Turn)')
ax4.grid(True, alpha=0.3)
ax4.legend()

# Adjust spacing between subplots to prevent text overlap
plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.93, 
                    wspace=0.25, hspace=0.35)
plt.savefig('OptimizedSpeeds_complete_analysis.png', dpi=300, bbox_inches='tight')
plt.show()

print(f'Complete analysis visualization saved as OptimizedSpeeds_complete_analysis.png')
print(f'Speed range: {min_speed:.1f} - {max_speed:.1f} mph (avg: {avg_speed:.1f} mph)')
