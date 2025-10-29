import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def parse_double_with_fallback(value, fallback=0.0, context=""):
    """
    Parse a string value to double with fallback on failure.
    
    Args:
        value: Value to parse
        fallback (float): Fallback value if parsing fails
        context (str): Context description for warning messages
    
    Returns:
        float: Parsed value or fallback
    """
    try:
        return float(value)
    except (ValueError, TypeError):
        if context:
            print(f"Warning: Could not parse '{value}' as double for {context}, using {fallback}")
        return fallback

def load_track_data(csv_path):
    """
    Load track data from CSV file with automatic coordinate format detection.
    Supports both X/Y coordinate format and Latitude/Longitude format with conversion.
    
    Args:
        csv_path (str): Path to the CSV file
    
    Returns:
        tuple: (x_coords, y_coords, distances) as numpy arrays
    """
    try:
        # Read CSV file
        df = pd.read_csv(csv_path)
        
        # Detect coordinate format from header
        header_lower = [col.strip().lower() for col in df.columns]
        x_index = -1
        y_index = -1
        is_lat_lon = False
        
        # Find X/Y or Lat/Lon columns
        for i, col in enumerate(header_lower):
            if col == 'x' or col == 'lat':
                x_index = i
            if col == 'y' or col == 'lon':
                y_index = i
            if col == 'lon':
                is_lat_lon = True
        
        if x_index == -1 or y_index == -1:
            raise ValueError("File header must contain 'X' and 'Y' columns or 'Lat' and 'Lon' columns.")
        
        print("Detected Latitude/Longitude format" if is_lat_lon else "Detected X/Y coordinate format")
        
        # Extract coordinate data with robust parsing
        raw_x = np.array([parse_double_with_fallback(val, 0.0, f"X at row {i+1}") 
                         for i, val in enumerate(df.iloc[:, x_index].values)])
        raw_y = np.array([parse_double_with_fallback(val, 0.0, f"Y at row {i+1}") 
                         for i, val in enumerate(df.iloc[:, y_index].values)])
        
        # Get reference point (first data point)
        x0 = raw_x[0]
        y0 = raw_y[0]
        
        # Convert coordinates to local coordinate system
        if is_lat_lon:
            # Convert lat/lon to meters using approximate conversion
            # 113200 meters per degree latitude
            # Longitude adjusted by cosine of latitude
            x_coords = 113200 * (raw_x - x0)
            y_coords = 113200 * np.cos(y0 * np.pi / 180) * (raw_y - y0)
            print(f"Converted from Lat/Lon using reference point: Lat={x0:.6f}, Lon={y0:.6f}")
        else:
            # Already in X/Y format, just offset to origin
            x_coords = raw_x - x0
            y_coords = raw_y - y0
            print(f"Offset coordinates using reference point: X={x0:.6f}, Y={y0:.6f}")
        
        # Calculate distances if not already present, or use existing column
        if df.shape[1] >= 3:
            # Try to use existing distance column
            distances = df.iloc[:, 2].values
            print("Using existing cumulative distance data")
        else:
            # Calculate cumulative distances
            distances = np.zeros(len(x_coords))
            for i in range(1, len(x_coords)):
                dx = x_coords[i] - x_coords[i-1]
                dy = y_coords[i] - y_coords[i-1]
                segment_distance = np.sqrt(dx*dx + dy*dy)
                distances[i] = distances[i-1] + segment_distance
            print("Calculated cumulative distances")
        
        print(f"Loaded {len(x_coords)} track points")
        print(f"Track range: X[{x_coords.min():.2f}, {x_coords.max():.2f}], Y[{y_coords.min():.2f}, {y_coords.max():.2f}]")
        print(f"Total track length: {distances.max():.2f} meters")
        
        return x_coords, y_coords, distances
        
    except FileNotFoundError:
        print(f"Error: Could not find file at {csv_path}")
        return None, None, None
    except Exception as e:
        print(f"Error loading data: {e}")
        return None, None, None

def plot_track_scatter(x_coords, y_coords, distances):
    """
    Create scatter plot of track coordinates colored by distance.
    
    Args:
        x_coords (np.array): X coordinates
        y_coords (np.array): Y coordinates  
        distances (np.array): Cumulative distances for coloring
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Track path colored by distance with connected line
    # First plot the connecting line
    ax1.plot(x_coords, y_coords, 'k-', linewidth=0.5, alpha=0.3, label='Track path')
    # Then plot the scatter points on top
    scatter = ax1.scatter(x_coords, y_coords, c=distances, cmap='viridis', 
                         s=1, alpha=0.8)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Track Layout - Colored by Distance (Connected)')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal', adjustable='box')
    ax1.legend()
    
    # Add colorbar for distance
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Cumulative Distance (m)')
    
    # Plot 2: Simple X-Y view with connected line
    ax2.plot(x_coords, y_coords, 'b-', linewidth=1, alpha=0.7, label='Track path')
    ax2.scatter(x_coords, y_coords, c='blue', s=1, alpha=0.6)
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title('Track Layout - Connected Path')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal', adjustable='box')
    ax2.legend()
    
    plt.tight_layout()
    
    # Save the plot
    plt.savefig('track_visualization.png', dpi=300, bbox_inches='tight')
    print("Track visualization saved as 'track_visualization.png'")
    
    plt.show()

def plot_distance_analysis(distances):
    """
    Create additional plots to analyze distance data.
    
    Args:
        distances (np.array): Cumulative distances
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    
    # Plot distance progression
    ax1.plot(distances, 'b-', linewidth=1)
    ax1.set_xlabel('Point Index')
    ax1.set_ylabel('Cumulative Distance (m)')
    ax1.set_title('Distance Progression Along Track')
    ax1.grid(True, alpha=0.3)
    
    # Plot distance increments (showing floating-point precision errors)
    distance_increments = np.diff(distances)
    ax2.plot(distance_increments, 'r-', linewidth=1)
    ax2.set_xlabel('Point Index')
    ax2.set_ylabel('Distance Increment (m)')
    ax2.set_title('Distance Between Consecutive Points\n(Shows Floating-Point Precision Errors)')
    ax2.grid(True, alpha=0.3)
    
    # Add text showing the actual range
    increment_range = distance_increments.max() - distance_increments.min()
    ax2.text(0.02, 0.95, f'Range: {increment_range:.2e} m\n(Target: 0.01 m)', 
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    
    # Save the distance analysis plot
    plt.savefig('distance_analysis.png', dpi=300, bbox_inches='tight')
    print("Distance analysis saved as 'distance_analysis.png'")
    
    plt.show()

def load_raw_track_data(csv_path):
    """
    Load and process raw track data from CSV file with coordinate detection and conversion.
    This function mirrors the C# LoadTrackData logic more closely.
    
    Args:
        csv_path (str): Path to the CSV file
    
    Returns:
        tuple: (track_data_dict, coordinate_info) where track_data_dict contains
               processed coordinates and coordinate_info contains metadata
    """
    try:
        # Read all lines from file
        with open(csv_path, 'r') as file:
            lines = file.readlines()
        
        if len(lines) < 2:
            raise ValueError("File must contain at least a header and one data row")
        
        data_rows = len(lines) - 1
        
        # Parse header to detect coordinate format
        header_values = [col.strip().lower() for col in lines[0].split(',')]
        x_index = -1
        y_index = -1
        is_lat_lon = False
        
        for i, col in enumerate(header_values):
            if col == 'x' or col == 'lat':
                x_index = i
            if col == 'y' or col == 'lon':
                y_index = i
            if col == 'lon':
                is_lat_lon = True
        
        if x_index == -1 or y_index == -1:
            raise ValueError("File header must contain 'X' and 'Y' columns or 'Lat' and 'Lon' columns.")
        
        print("Detected Latitude/Longitude format" if is_lat_lon else "Detected X/Y coordinate format")
        
        # Get reference point from first data row
        first_data_values = lines[1].split(',')
        x0 = parse_double_with_fallback(first_data_values[x_index], 0.0, "X at line 1")
        y0 = parse_double_with_fallback(first_data_values[y_index], 0.0, "Y at line 1")
        
        # Initialize arrays
        track_points = []
        
        # Parse data rows
        for i in range(1, len(lines)):
            values = lines[i].split(',')
            if len(values) < 2:
                continue
                
            raw_x = parse_double_with_fallback(values[x_index], 0.0, f"X at line {i}")
            raw_y = parse_double_with_fallback(values[y_index], 0.0, f"Y at line {i}")
            
            # Convert coordinates based on format
            if is_lat_lon:
                # Convert lat/lon to meters (matches C# logic exactly)
                x = 113200 * (raw_x - x0)
                y = 113200 * np.cos(y0 * np.pi / 180) * (raw_y - y0)
            else:
                # Simple offset for X/Y coordinates
                x = raw_x - x0
                y = raw_y - y0
            
            track_points.append({
                'X': x,
                'Y': y,
                'CumulativeDistance': 0.0,  # Will be calculated later
                'Curvature': 0.0,          # Will be calculated later
                'TangentDirection': 0.0    # Will be calculated later
            })
        
        # Calculate cumulative distances
        for i in range(1, len(track_points)):
            dx = track_points[i]['X'] - track_points[i-1]['X']
            dy = track_points[i]['Y'] - track_points[i-1]['Y']
            segment_distance = np.sqrt(dx*dx + dy*dy)
            track_points[i]['CumulativeDistance'] = track_points[i-1]['CumulativeDistance'] + segment_distance
        
        coordinate_info = {
            'is_lat_lon': is_lat_lon,
            'reference_point': (x0, y0),
            'x_index': x_index,
            'y_index': y_index,
            'total_points': len(track_points)
        }
        
        print(f"Processed {len(track_points)} track points")
        if track_points:
            print(f"Total track length: {track_points[-1]['CumulativeDistance']:.2f} meters")
        
        return track_points, coordinate_info
        
    except FileNotFoundError:
        print(f"Error: Could not find file at {csv_path}")
        return None, None
    except Exception as e:
        print(f"Error processing raw data: {e}")
        return None, None

def main():
    """Main function to run the track visualizer."""
    print("Track Visualizer - Enhanced with C# Logic")
    print("=" * 45)
    
    # Path to the CSV file (relative to current directory)
    csv_file = input("Enter track data file path.\n[Default: '../InputData/SkidPadTrackMap.csv']:")
    if not csv_file:
        csv_file = "../InputData/SkidPadTrackMap.csv"
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"File not found: {csv_file}")
        print("Please ensure the track CSV file exists in the specified path.")
        return
    
    # Choose loading method
    print("\nSelect loading method:")
    print("1. Pandas-based loader (default)")
    print("2. Raw CSV processor (mirrors C# logic)")
    choice = input("Enter choice [1]: ").strip()
    
    if choice == "2":
        # Use raw CSV processor
        track_points, coord_info = load_raw_track_data(csv_file)
        if track_points is None:
            return
            
        # Convert to arrays for plotting
        x_coords = np.array([point['X'] for point in track_points])
        y_coords = np.array([point['Y'] for point in track_points])
        distances = np.array([point['CumulativeDistance'] for point in track_points])
        
        print(f"\nCoordinate format: {'Lat/Lon' if coord_info['is_lat_lon'] else 'X/Y'}")
        print(f"Reference point: {coord_info['reference_point']}")
    else:
        # Use pandas-based loader (default)
        x_coords, y_coords, distances = load_track_data(csv_file)
        if x_coords is None:
            return
    
    # Create scatter plots
    plot_track_scatter(x_coords, y_coords, distances)
    
    # Create distance analysis plots  
    plot_distance_analysis(distances)
    
    # Print some statistics
    distance_increments = np.diff(distances)
    print("\nTrack Statistics:")
    print(f"Number of points: {len(x_coords)}")
    print(f"X range: {x_coords.min():.2f} to {x_coords.max():.2f} meters")
    print(f"Y range: {y_coords.min():.2f} to {y_coords.max():.2f} meters")
    print(f"Track width (X): {x_coords.max() - x_coords.min():.2f} meters")
    print(f"Track height (Y): {y_coords.max() - y_coords.min():.2f} meters")
    print(f"Total track length: {distances.max():.2f} meters")
    if len(distance_increments) > 0:
        print(f"Average point spacing: {distances.max()/len(x_coords):.4f} meters")
        print(f"Actual spacing range: {distance_increments.min():.15f} to {distance_increments.max():.15f} meters")
        print(f"Spacing variation: ±{(distance_increments.max() - distance_increments.min())/2:.2e} meters")

if __name__ == "__main__":
    main()
