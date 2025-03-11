# Right Wall Follower Robot

## Project Overview

This project implements a robot that navigates through maze environments by following the right wall. The robot uses a combination of sensors to detect walls and obstacles, maintaining a consistent distance from the right wall while avoiding collisions.

The main goal was to create a reliable navigation algorithm that could handle various maze configurations without getting stuck or colliding with walls. The robot uses a combination of proportional control for wall following and reactive obstacle avoidance for navigation.

## Key Features

- **Reliable Right Wall Following**: Maintains consistent distance from the right wall
- **Multi-Sensor Obstacle Detection**: Uses 6 strategically placed sensors for comprehensive environment perception
- **Adaptive Speed Control**: Automatically slows down when approaching obstacles or navigating turns
- **Parallel Wall Alignment**: Actively works to stay parallel to the wall for smoother movement
- **Conservative Movement Parameters**: Prioritizes safety and reliability over speed
- **Visual Feedback**: Displays current status and sensor readings for debugging

## Key Challenges and Solutions

### Challenge 1: Consistent Wall Following

**Problem**: The robot needed to maintain a fixed distance from the right wall without oscillating or drifting.

**Solution**: Implemented a dual-component control system that:
1. Uses proportional control to maintain ideal distance from the wall
2. Calculates wall angle to ensure the robot stays parallel to the wall
3. Applies variable smoothing for stable movement

### Challenge 2: Collision Avoidance

**Problem**: Initial implementations would occasionally bump into walls, especially in tight corners or complex maze sections.

**Solution**:
1. Added multiple front-facing sensors with different angles for better obstacle detection
2. Implemented conservative speed parameters to ensure the robot could always stop in time
3. Created safety override that stops forward movement when obstacles are too close
4. Added increased detection distances to begin turning well before reaching obstacles

### Challenge 3: Navigation Around Corners

**Problem**: The robot would often take wide turns around corners, making it difficult to navigate narrow mazes.

**Solution**:
1. Added specific front-right sensor to detect approaching right corners
2. Implemented adaptive turning based on which sensors detect obstacles
3. Gradually reduces speed when approaching corners for more precise navigation

## Installation and Setup

### Prerequisites

- Docker installed on your system
- Git for cloning the repository

### Installation Steps

1. Clone the repository:
   ```
   git clone https://github.com/jstkyle/MazeBot.git
   cd MazeBot
   ```

2. Build and run the Docker container:
   ```
   docker build -t MazeBot .
   docker run -p 8765:8765 -v $PWD:/source -it MazeBot
   ```

3. In a new terminal window, start the enviro server:
   ```
   docker exec -it $(docker ps | grep MazeBot | awk '{print $1}') bash -c "cd /source && enviro"
   ```

4. Open a web browser and navigate to:
   ```
   http://localhost
   ```

## Usage Guide

### Running the Simulation

1. The robot will automatically start following the right wall once the simulation begins
2. The robot maintains an ideal distance of approximately 30 units from the right wall
3. When the robot encounters an obstacle in front, it will turn left to avoid it
4. If the robot loses track of the right wall, it will turn right to find it again

### Testing in Different Maze Configurations

The robot has been tested in various maze configurations, including:
- Long straight corridors
- Right angle turns
- Sharp corners
- Dead ends
- Narrow passages

## Code Structure

The project consists of two main files:

1. **my_robot.h**: Contains the controller logic for the robot
   - `init()`: Initializes the robot controller
   - `update()`: Main control loop that processes sensor inputs and controls movement
   
2. **my_robot.json**: Defines the physical properties and sensor configuration
   - Robot shape and mass
   - Friction parameters
   - Sensor positions and orientations

## Future Improvements

Potential enhancements for future versions:

1. **Mapping capabilities**: Allow the robot to build a map of the maze as it explores
2. **Path optimization**: Implement algorithms to find shorter paths after initial exploration
3. **Dynamic parameter adjustment**: Automatically tune control parameters based on maze characteristics
4. **Multi-robot coordination**: Enable multiple robots to explore cooperatively

## Acknowledgments

- This project was built using the Enviro framework: https://github.com/klavinslab/enviro
- Inspiration and algorithms based on robotics literature on wall-following techniques
- Special thanks to the course instructor and TAs for their guidance

## License

This project is licensed under the MIT License - see the LICENSE file for details.