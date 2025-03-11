#ifndef __RIGHT_WALL_FOLLOWER__H
#define __RIGHT_WALL_FOLLOWER__H 

#include <string>
#include <math.h>
#include "enviro.h"
#include <chipmunk.h> // Include Chipmunk2D header for cpVect

using namespace enviro;

class RightWallFollowerController : public Process, public AgentInterface {
    public:
        RightWallFollowerController() : Process(), AgentInterface(), prev_right_avg(0), at_goal(false) {}

        void init() {
            std::cout << "Enhanced Right Wall Follower initialized\n";
            label("Initialized", 0, -20); // Initial label
        }

        void start() {}

        void update() {
            // Get sensor readings
            double front_center = sensor_value(0);     // Front center sensor
            double front_left = sensor_value(1);       // Front left sensor
            double front_right = sensor_value(2);      // Front right sensor
            double right_front = sensor_value(3);      // Right front sensor
            double right_middle = sensor_value(4);     // Right middle sensor
            double right_back = sensor_value(5);       // Right back sensor
            
            // Default movement values
            double forward_speed = 0;
            double turn_rate = 0;
            
            // Parameters
            const double IDEAL_DISTANCE = 30;        // Ideal distance from right wall
            const double FRONT_THRESHOLD = 60;       // Distance to detect front obstacles
            const double CRITICAL_THRESHOLD = 30;    // Emergency stop distance
            const double MAX_SPEED = 5;              // Modest max speed
            const double WALL_END_THRESHOLD = 60;    // Threshold for detecting wall end
            const double SUDDEN_INCREASE_THRESHOLD = 20; // Minimum increase to detect wall end
            const double MAX_TURN_RATE = 50.0;       // Max turn rate for fast turns
            const double TURN_SENSITIVITY = 2.0;     // Sensitivity for dynamic turn
            const double MIN_TURN_RATE = 20.0;       // Minimum turn rate during wall-end turn
            const double GOAL_X = 340.0;             // Goal x-coordinate
            const double GOAL_Y = 260.0;             // Goal y-coordinate
            const double GOAL_THRESHOLD = 120.0;     // Widened distance threshold to consider robot at goal
            
            // Declare and initialize smoothing factor
            double smoothing_factor = 0.8; // Default smoothing factor
            
            // Get current position using cpVect
            cpVect p = position();  // position() should return cpVect
            double current_x = p.x;
            double current_y = p.y;
            
            // Check if robot is at goal
            double distance_to_goal = sqrt(pow(current_x - GOAL_X, 2) + pow(current_y - GOAL_Y, 2));
            if (distance_to_goal < GOAL_THRESHOLD && !at_goal) {
                at_goal = true;
                std::cout << "Goal Reached at (" << current_x << ", " << current_y << ")!\n";
            }
            
            // Find the minimum front sensor reading (for obstacle detection)
            double min_front = std::min(std::min(front_center, front_left), front_right);
            
            // Calculate average right distance for wall following
            double right_avg = (right_front + right_middle + right_back) / 3.0;
            
            // Calculate if we're parallel to the wall
            double wall_angle = 0;
            if (right_front < 100 && right_back < 100) {
                wall_angle = atan2(right_front - right_back, 16); // 16 is approx distance between sensors
            }
            
            // Status for debugging
            std::string status = "";
            
            // Detect sudden wall end
            bool wall_ended = (right_avg > WALL_END_THRESHOLD) && 
                             (prev_right_avg > 0) && 
                             (right_avg - prev_right_avg > SUDDEN_INCREASE_THRESHOLD);
            
            // Calculate dynamic turn rate based on wall loss speed
            double distance_change = right_avg - prev_right_avg;
            double dynamic_turn_rate = 0.0;
            if (wall_ended && !at_goal) {
                dynamic_turn_rate = distance_change * TURN_SENSITIVITY; // Proportional to wall loss speed
                if (dynamic_turn_rate > MAX_TURN_RATE) dynamic_turn_rate = MAX_TURN_RATE; // Cap the turn rate
                if (dynamic_turn_rate < MIN_TURN_RATE) dynamic_turn_rate = MIN_TURN_RATE; // Ensure minimum turn rate
                turn_rate = dynamic_turn_rate; // Apply the calculated turn rate
                std::cout << "Wall ended - Change: " << distance_change << ", Turn Rate: " << turn_rate 
                          << ", Angular Vel: " << angular_velocity() << " rad/s" << std::endl;
            }
            
            // SAFETY CHECK: Prevent collisions
            if (min_front < CRITICAL_THRESHOLD && !at_goal) {
                forward_speed = 0;
                turn_rate = -0.6;  // Turn left to avoid obstacle
                status = "SAFETY STOP";
                smoothing_factor = 0.5; // Quick response for safety
            }
            // If there's an obstacle ahead, start turning left
            else if (min_front < FRONT_THRESHOLD && !at_goal) {
                double turn_factor = 1.0 - (min_front / FRONT_THRESHOLD);
                bool obstacle_right = (front_right < front_left);
                turn_rate = obstacle_right ? -0.5 - turn_factor * 0.3 : -0.4 - turn_factor * 0.3;
                status = obstacle_right ? "Front-Right Obstacle: Sharp Left" : "Front Obstacle: Turn Left";
                forward_speed = MAX_SPEED * (1.0 - turn_factor * 0.8);
            }
            // Normal wall-following behavior
            else {
                if (wall_ended && !at_goal) {
                    forward_speed = MAX_SPEED * 0.2; // Reduced speed during turn
                    status = "Dynamic Right Turn (Wall End) - Turn Rate: " + std::to_string(int(turn_rate));
                    smoothing_factor = 0.0; // No smoothing for immediate response
                } else if (right_avg > IDEAL_DISTANCE * 2 || right_avg == 0 && !at_goal) {
                    forward_speed = MAX_SPEED * 0.6; // Reduce speed while searching
                    turn_rate = 0.5; // Gentle right turn to find wall
                    status = "Searching for Wall";
                } else if (!at_goal) {
                    // Normal right wall following
                    double distance_error = IDEAL_DISTANCE - right_avg;
                    turn_rate = (wall_angle * 1.5) + (distance_error * 0.02);
                    if (turn_rate > 0.5) turn_rate = 0.5;
                    if (turn_rate < -0.5) turn_rate = -0.5;
                    double turn_penalty = std::abs(turn_rate) * 1.2;
                    double alignment_penalty = std::abs(wall_angle) * 2.0;
                    forward_speed = MAX_SPEED * (1.0 - turn_penalty - alignment_penalty);
                    if (forward_speed < 2) forward_speed = 2;
                    if (std::abs(distance_error) < 5 && std::abs(wall_angle) < 0.1) {
                        status = "Optimal Wall Following";
                    } else if (std::abs(wall_angle) > 0.1) {
                        status = wall_angle > 0 ? "Adjusting Angle: Diverging" : "Adjusting Angle: Converging";
                    } else {
                        status = distance_error > 0 ? "Too Far: Moving Right" : "Too Close: Moving Left";
                    }
                } else {
                    // At goal: stop moving
                    forward_speed = 0;
                    turn_rate = 0;
                    status = "GOAL REACHED - STOPPED";
                }
            }
            
            // Apply movement smoothing
            current_speed = current_speed * smoothing_factor + forward_speed * (1.0 - smoothing_factor);
            current_turn = current_turn * smoothing_factor + turn_rate * (1.0 - smoothing_factor);
            
            // Apply calculated movement
            track_velocity(current_speed, current_turn);
            
            // Display status and sensor readings
            if (!at_goal) {
                label(status, 0, -20);
                std::string front_info = "Front: " + std::to_string(int(front_center)) + 
                                       " | L: " + std::to_string(int(front_left)) +
                                       " | R: " + std::to_string(int(front_right));
                std::string right_info = "Right: " + std::to_string(int(right_front)) + 
                                       " | " + std::to_string(int(right_middle)) + 
                                       " | " + std::to_string(int(right_back));
                label(front_info, 0, -35);
                label(right_info, 0, -50);
            } else {
                // Display prominent "GOAL REACHED!" message when at goal
                label("GOAL REACHED!", 0, -20); // Larger offset to make it more visible
                label("Final Position: (" + std::to_string(int(current_x)) + ", " + 
                      std::to_string(int(current_y)) + ")", 0, -40); // Additional info
            }
            
            // Update previous right average for next iteration
            prev_right_avg = right_avg;
        }

        void stop() {}

    private:
        double current_speed = 0;
        double current_turn = 0;
        double prev_right_avg = 0;  // Previous right average for detecting sudden changes
        bool at_goal = false;       // Flag to track if goal is reached
};

class RightWallFollower : public Agent {
    public:
        RightWallFollower(json spec, World& world) : Agent(spec, world) {
            add_process(controller);
        }
        
    private:
        RightWallFollowerController controller;
};

DECLARE_INTERFACE(RightWallFollower)

#endif