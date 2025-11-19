import matplotlib.pyplot as plt
import pandas as pd
import sys
import os

def plot_data(df, filename):
    plt.figure(figsize=(16, 12))
    
    # 1. Altitude vs. Time
    plt.subplot(3, 3, 1)
    plt.plot(df['time(s)'], df['CoordinateOz(m)'], 'b-', linewidth=2)
    plt.title('Altitude vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.grid(True, alpha=0.3)
    
    # 2. Velocity Components vs. Time
    plt.subplot(3, 3, 2)
    plt.plot(df['time(s)'], df['velocityOx(m/s)'], 'r-', label='Velocity X', linewidth=2)
    plt.plot(df['time(s)'], df['velocityOy(m/s)'], 'g-', label='Velocity Y', linewidth=2)
    plt.plot(df['time(s)'], df['velocityOz(m/s)'], 'b-', label='Velocity Z', linewidth=2)
    plt.title('Velocity Components vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 3. Acceleration Components vs. Time
    plt.subplot(3, 3, 3)
    plt.plot(df['time(s)'], df['accOx(m/s^2)'], 'r-', label='Acceleration X', linewidth=2)
    plt.plot(df['time(s)'], df['accOy(m/s^2)'], 'g-', label='Acceleration Y', linewidth=2)
    plt.plot(df['time(s)'], df['accOz(m/s^2)'], 'b-', label='Acceleration Z', linewidth=2)
    plt.title('Acceleration Components vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 4. Engine Thrust vs. Time
    plt.subplot(3, 3, 4)
    plt.plot(df['time(s)'], df['thrust_percent()'], 'purple', linewidth=2)
    plt.title('Engine Thrust vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust (%)')
    plt.grid(True, alpha=0.3)

    # 5. Fuel Mass vs. Time
    plt.subplot(3, 3, 5)
    plt.plot(df['time(s)'], df['fuel_mass(kg)'], 'orange', linewidth=2)
    plt.title('Fuel Mass vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Fuel Mass (kg)')
    plt.grid(True, alpha=0.3)
    
    # 6. Total Velocity vs. Time
    plt.subplot(3, 3, 6)
    total_velocity = (df['velocityOx(m/s)']**2 + df['velocityOy(m/s)']**2 + df['velocityOz(m/s)']**2)**0.5
    plt.plot(df['time(s)'], total_velocity, 'red', linewidth=2)
    plt.title('Total Velocity vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True, alpha=0.3)
    
    # 7. Total Acceleration vs. Time
    plt.subplot(3, 3, 7)
    total_acceleration = (df['accOx(m/s^2)']**2 + df['accOy(m/s^2)']**2 + df['accOz(m/s^2)']**2)**0.5
    plt.plot(df['time(s)'], total_acceleration, 'darkblue', linewidth=2)
    plt.title('Total Acceleration vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    base_filename, _ = os.path.splitext(filename)
    output_filename = base_filename + '.png'
    plt.savefig(output_filename)
    print(f"Plot saved to {output_filename}")


def flight_summary(df):
    max_total_velocity = (df['velocityOx(m/s)']**2 + df['velocityOy(m/s)']**2 + df['velocityOz(m/s)']**2)**0.5
    max_total_acceleration = (df['accOx(m/s^2)']**2 + df['accOy(m/s^2)']**2 + df['accOz(m/s^2)']**2)**0.5
   
    print("\nFlight Summary:")
    print(f"  Total flight time: {df['time(s)'].max():.2f} s")
    print(f"  Maximum speed: {max_total_velocity.max():.2f} m/s")
    print(f"  Maximum acceleration: {max_total_acceleration.max():.2f} m/s²")
    print(f"  Fuel used: {(df['fuel_mass(kg)'].iloc[0] - df['fuel_mass(kg)'].iloc[-1]):.2f} kg")

# Main program
if __name__ == "__main__":
    if len(sys.argv) < 2:
       print("Usage: python3 stats.py <path_to_csv_file>")
       sys.exit(1) 

    filename = sys.argv[1] 
    if not os.path.exists(filename):
        print(f"Error: File not found at '{filename}'")
        sys.exit(1)

    try:
        df = pd.read_csv(filename)
    except Exception as e:
        print(f"Error reading or parsing CSV file: {e}")
        sys.exit(1)
    
    # Generate plot
    plot_data(df, filename)

    # Print flight summary
    flight_summary(df)