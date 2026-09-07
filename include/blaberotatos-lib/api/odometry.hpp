#pragma once

#include "blaberotatos-lib/api/drivetrain.hpp"

#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

#include <cmath>

class odometry
{
    private:
        // References to physical sensors
        pros::Rotation& parallel_tracker_; // Parallel to the robot's direction of movement
            // Also update the other classes to using the naming convention with a trailing underscore
        pros::Rotation& perpendicular_tracker_;
        pros::Imu& imu_;

        // Measurements of physical setup
        double tracking_wheel_circumference_;
        double parallel_tracker_offset_;
        double perpendicular_tracker_offset_;

        // Past sensor measurements
        double previous_parallel_tracker_pose_ = 0; // Initialize them to 0 at the start
        double previous_perpendicular_tracker_pose_ = 0;
        double previous_imu_reading_ = 0; // In radians
        
        // Coordinates
        double x_; // In inches
        double y_; // In inches
        double theta_; // In radians

    public:
        // Constructor
        odometry(pros::Rotation& parallel_tracker,
                 pros::Rotation& perpendicular_tracker,
                 pros::Imu& imu,
                 double tracking_wheel_diameter,
                 double parallel_tracker_offset,
                 double perpendicular_tracker_offset);

        // Background loop functions
        void set_pose(double new_x, double new_y, double new_theta_degrees);
        void update_position();

        // Getters so that the autonomous routine can read the coordinates
        double get_x() const {return x_;}
        double get_y() const {return y_;}
        double get_theta_radians() const {return theta_;}
        double get_theta_degrees() const {return theta_ * (180.0 / M_PI);}
    };