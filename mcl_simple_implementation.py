import numpy as np
import matplotlib.pyplot as plt

# Map size and number of particles
MAP_SIZE = (100, 100)
NUM_PARTICLES = 500

# Robot's true position (x, y, theta)
TRUE_POSITION = [50, 50, np.pi / 4]  # Robot starts at center, facing 45 degrees

# Motion and sensor noise
MOTION_NOISE = 2.0  # Uncertainty in movement
SENSOR_NOISE = 5.0  # Uncertainty in sensor readings

# Sensor model: Assuming robot senses distance to a landmark
LANDMARK = [80, 80]  # Position of a landmark

def initialize_particles(num_particles, map_size):
    """Initialize particles uniformly over the map."""
    particles = np.empty((num_particles, 3))  # (x, y, theta)
    particles[:, 0] = np.random.uniform(0, map_size[0], num_particles)  # x
    particles[:, 1] = np.random.uniform(0, map_size[1], num_particles)  # y
    particles[:, 2] = np.random.uniform(0, 2 * np.pi, num_particles)    # theta
    return particles

def move_particles(particles, delta_pos):
    """Move particles based on robot's motion with added noise."""
    dx, dy, dtheta = delta_pos
    particles[:, 0] += dx + np.random.normal(0, MOTION_NOISE, len(particles))
    particles[:, 1] += dy + np.random.normal(0, MOTION_NOISE, len(particles))
    particles[:, 2] += dtheta + np.random.normal(0, MOTION_NOISE, len(particles))
    return particles

def calculate_sensor_model(particles, landmark):
    """Calculate weights based on sensor measurements."""
    # True sensor measurement (distance to the landmark)
    true_distance = np.sqrt((TRUE_POSITION[0] - landmark[0])**2 + (TRUE_POSITION[1] - landmark[1])**2)  # TODO(filip): how true position
    
    # Particles' distances to the landmark
    distances = np.sqrt((particles[:, 0] - landmark[0])**2 + (particles[:, 1] - landmark[1])**2)
    
    # Assign weights based on Gaussian probability
    weights = np.exp(- ((distances - true_distance)**2) / (2 * SENSOR_NOISE**2))
    weights += 1.e-300  # Avoid zero weights
    return weights

def resample_particles(particles, weights):  # TODO(filip): 
    """Resample particles based on their weights."""
    indices = np.random.choice(len(particles), size=len(particles), p=weights / np.sum(weights))
    return particles[indices]

def estimate_position(particles):
    """Estimate position as the average of the particles."""
    x = np.mean(particles[:, 0])
    y = np.mean(particles[:, 1])
    theta = np.mean(particles[:, 2])
    return [x, y, theta]

def plot_particles(particles, true_position, estimated_position):
    """Plot particles, true position, and estimated position."""
    plt.clf()
    plt.scatter(particles[:, 0], particles[:, 1], color='blue', s=2, label='Particles')
    plt.scatter(true_position[0], true_position[1], color='green', s=100, label='True Position')
    plt.scatter(estimated_position[0], estimated_position[1], color='red', s=100, label='Estimated Position')
    plt.xlim(0, MAP_SIZE[0])
    plt.ylim(0, MAP_SIZE[1])
    plt.legend()
    plt.pause(0.1)

# Main Monte Carlo Localization Loop
def monte_carlo_localization():
    particles = initialize_particles(NUM_PARTICLES, MAP_SIZE)

    for t in range(10):  # Simulate 20 time steps
        # Simulated robot motion (move 5 units forward)
        delta_pos = [0.0, 0.0, 0.0]  # u -> control
        move_particles(particles, delta_pos)

        # Calculate weights based on sensor readings
        weights = calculate_sensor_model(particles, LANDMARK)

        # Resample particles based on their weights
        particles = resample_particles(particles, weights)  # TODO(filip): 

        # Estimate position
        estimated_position = estimate_position(particles)
        print(f"Step {t+1}: Estimated Position: {estimated_position}")

        # Plot results
        TRUE_POSITION[0] += estimated_position[0]
        TRUE_POSITION[1] += estimated_position[1]
        TRUE_POSITION[2] += estimated_position[2]
        plot_particles(particles, TRUE_POSITION, estimated_position)
    
    plt.show()

if __name__ == "__main__":
    monte_carlo_localization()
