import re
import matplotlib.pyplot as plt

def parse_odo_file(filename):
    """Extract ODO-X and ODO-Y data (unit: mm) from file"""
    x_coords = []
    y_coords = []
    pattern = re.compile(r'ODO-X: (\d+), ODO-Y: (\d+)')

    with open(filename, 'r') as file:
        for line in file:
            if '[ODOdata]' not in line:
                continue
            match = pattern.search(line)
            if match:
                x_mm, y_mm = map(int, match.groups())
                x_coords.append(x_mm)
                y_coords.append(y_mm)

    return x_coords, y_coords

def plot_trajectory(x_coords, y_coords):
    """Plot (x, y) trajectory"""
    plt.figure(figsize=(10, 6))

    # Plot trajectory line with direction indicators
    plt.plot(x_coords, y_coords, 'b-', linewidth=1, alpha=0.5, label='Path')
    plt.plot(x_coords, y_coords, 'r.', markersize=5, label='Data points')

    # Add direction arrows (sampled to avoid clutter)
    step = max(1, len(x_coords)//20)
    for i in range(0, len(x_coords)-1, step):
        dx = x_coords[i+1] - x_coords[i]
        dy = y_coords[i+1] - y_coords[i]
        plt.arrow(x_coords[i], y_coords[i], dx, dy,
                 shape='full', color='green', length_includes_head=True,
                 head_width=5, head_length=10, alpha=0.7)

    # Mark start and end points
    plt.scatter(x_coords[0], y_coords[0], c='green', s=100, marker='o', label='Start')
    plt.scatter(x_coords[-1], y_coords[-1], c='red', s=100, marker='X', label='End')

    plt.title('Robot Trajectory (mm)')
    plt.xlabel('X coordinate (mm)')
    plt.ylabel('Y coordinate (mm)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    try:
        x, y = parse_odo_file('odo.txt')
        if not x:
            print("Error: No valid ODO data found!")
        else:
            print(f"Successfully read {len(x)} data points")
            print(f"X range: {min(x)}mm → {max(x)}mm")
            print(f"Y range: {min(y)}mm → {max(y)}mm")
            plot_trajectory(x, y)
    except FileNotFoundError:
        print("Error: odo.txt file not found")