#!/usr/bin/env python3
"""
Simple script to plot velocity (vx, vy) and position (x, y) data from the radar odometry module.
Creates two figures: 1) velocity plot, 2) position plot
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def plot_velocity_and_position():
    """Load and plot both velocity and position data."""
    
    # Load velocity data
    velocity_path = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/output/velocity_data.csv"
    position_path = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/output/odometry_estimate_xy.csv"
    
    # Check if files exist
    if not os.path.exists(velocity_path):
        print(f"Error: Velocity CSV file not found at {velocity_path}")
        return
    
    if not os.path.exists(position_path):
        print(f"Error: Position CSV file not found at {position_path}")
        return
    
    # Load data
    velocity_df = pd.read_csv(velocity_path)
    position_df = pd.read_csv(position_path)
    
    print(f"Loaded {len(velocity_df)} velocity data points")
    print(f"Loaded {len(position_df)} position data points")
    
    # Figure 1: Velocity Plot (vx, vy) in 2x1 format
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Plot vx
    ax1.plot(velocity_df['vx'].values, 'b-', linewidth=1.5, label='vx')
    ax1.set_ylabel('vx (m/s)')
    ax1.set_title('Velocity X Component (vx)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Add vx statistics
    ax1.text(0.02, 0.98, f'vx Max: {velocity_df["vx"].max():.3f} m/s\nvx Avg: {velocity_df["vx"].mean():.3f} m/s', 
            transform=ax1.transAxes, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=10)
    
    # Plot vy
    ax2.plot(velocity_df['vy'].values, 'r-', linewidth=1.5, label='vy')
    ax2.set_ylabel('vy (m/s)')
    ax2.set_xlabel('Sample Index')
    ax2.set_title('Velocity Y Component (vy)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Add vy statistics
    ax2.text(0.02, 0.98, f'vy Max: {velocity_df["vy"].max():.3f} m/s\nvy Avg: {velocity_df["vy"].mean():.3f} m/s', 
            transform=ax2.transAxes, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=10)
    
    plt.tight_layout()
    plt.savefig('velocity_plot.png', dpi=300, bbox_inches='tight')
    print("Saved velocity plot: velocity_plot.png")
    plt.show()
    
    # Figure 2: Position Plot (x, y) as 2D trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(position_df['x'].values, position_df['y'].values, 'g-', linewidth=1.5, alpha=0.7, label='Trajectory')
    plt.plot(position_df['x'].iloc[0], position_df['y'].iloc[0], 'go', markersize=10, label='Start')
    plt.plot(position_df['x'].iloc[-1], position_df['y'].iloc[-1], 'ro', markersize=10, label='End')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Robot Trajectory (x, y)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    # Add trajectory statistics
    dx = np.diff(position_df['x'].values)
    dy = np.diff(position_df['y'].values)
    distances = np.sqrt(dx**2 + dy**2)
    total_distance = np.sum(distances)
    displacement = np.sqrt((position_df['x'].iloc[-1] - position_df['x'].iloc[0])**2 + 
                          (position_df['y'].iloc[-1] - position_df['y'].iloc[0])**2)
    
    plt.text(0.02, 0.98, f'Total Distance: {total_distance:.2f} m\nDisplacement: {displacement:.2f} m\nEfficiency: {(displacement/total_distance)*100:.1f}%', 
            transform=plt.gca().transAxes, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=10)
    
    plt.tight_layout()
    plt.savefig('position_plot.png', dpi=300, bbox_inches='tight')
    print("Saved position plot: position_plot.png")
    plt.show()
    
    # Print basic statistics
    print(f"\nVelocity Statistics:")
    print(f"  vx: mean={velocity_df['vx'].mean():.3f}, std={velocity_df['vx'].std():.3f}, range=[{velocity_df['vx'].min():.3f}, {velocity_df['vx'].max():.3f}] m/s")
    print(f"  vy: mean={velocity_df['vy'].mean():.3f}, std={velocity_df['vy'].std():.3f}, range=[{velocity_df['vy'].min():.3f}, {velocity_df['vy'].max():.3f}] m/s")
    
    print(f"\nPosition Statistics:")
    print(f"  x: mean={position_df['x'].mean():.3f}, std={position_df['x'].std():.3f}, range=[{position_df['x'].min():.3f}, {position_df['x'].max():.3f}] m")
    print(f"  y: mean={position_df['y'].mean():.3f}, std={position_df['y'].std():.3f}, range=[{position_df['y'].min():.3f}, {position_df['y'].max():.3f}] m")

def main():
    """Main function."""
    print("Velocity and Position Plotting Script")
    print("=" * 40)
    plot_velocity_and_position()

if __name__ == "__main__":
    main()
