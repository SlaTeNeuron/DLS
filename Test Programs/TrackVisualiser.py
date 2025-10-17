import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def load_track_data(csv_path):
    """
    Load track data from CSV file and extract first 3 columns.
    
    Args:
        csv_path (str): Path to the CSV file
    
    Returns:
        tuple: (x_coords, y_coords, distances) as numpy arrays
    """
    try:
        # Read CSV file
        df = pd.read_csv(csv_path)
        
        # Extract first 3 columns: X, Y, CumulativeDistance
        x_coords = df.iloc[:, 0].values  # X coordinates
        y_coords = df.iloc[:, 1].values  # Y coordinates  
        distances = df.iloc[:, 2].values  # Cumulative Distance
        
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
    
    # Plot 1: Track path colored by distance
    scatter = ax1.scatter(x_coords, y_coords, c=distances, cmap='viridis', 
                         s=1, alpha=0.8)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Track Layout - Colored by Distance')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal', adjustable='box')
    
    # Add colorbar for distance
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Cumulative Distance (m)')
    
    # Plot 2: Simple X-Y scatter without coloring
    ax2.scatter(x_coords, y_coords, c='blue', s=1, alpha=0.6)
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title('Track Layout - Simple View')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal', adjustable='box')
    
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

def main():
    """Main function to run the track visualizer."""
    print("Track Visualizer - Skid Pad Analysis")
    print("=" * 40)
    
    # Path to the CSV file (relative to current directory)
    csv_file = "InputData/SkidPadTrack.csv"
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"File not found: {csv_file}")
        print("Please ensure the SkidPadTrack.csv file is in the InputData folder.")
        return
    
    # Load track data
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
    print(f"Target point spacing: 0.01 meters")
    print(f"Actual spacing range: {distance_increments.min():.15f} to {distance_increments.max():.15f} meters")
    print(f"Spacing variation: ±{(distance_increments.max() - distance_increments.min())/2:.2e} meters (floating-point precision errors)")

if __name__ == "__main__":
    main()
