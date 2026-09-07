#include "main.h"

#include "blaberotatos-lib/api/drivetrain.hpp"
#include "blaberotatos-lib/api/odometry.hpp"

#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

#include <cmath>

// How does constructor notation like this work?
odometry::odometry(pros::Rotation& parallel_tracker,
                   pros::Rotation& perpendicular_tracker,
                   pros::Imu& imu,
                   double tracking_wheel_diameter,
                   double parallel_tracker_offset,
                   double perpendicular_tracker_offset)
: parallel_tracker_(parallel_tracker),
  perpendicular_tracker_(perpendicular_tracker),
  imu_(imu),
  parallel_tracker_offset_(parallel_tracker_offset),
  perpendicular_tracker_offset_(perpendicular_tracker_offset),
  x_(0),
  y_(0),
  theta_(0)
{
    tracking_wheel_circumference_ = M_PI * tracking_wheel_diameter;
}

void odometry::set_pose(double new_x, double new_y, double new_theta_degrees)
{
    // Change coordinates
    x_ = new_x;
    y_ = new_y;
    theta_ = new_theta_degrees * (M_PI / 180.0);

    // Tracker readings are updated as normal -- they are a different layer than the final x and y coordinates
    previous_parallel_tracker_pose_ = parallel_tracker_.get_position() / 100.0 // centidegrees to degrees
                                      * tracking_wheel_circumference_ / 360.0; // degrees to inches
    previous_perpendicular_tracker_pose_ = perpendicular_tracker_.get_position() / 100.0 // centidegrees to degrees
                                           * tracking_wheel_circumference_ / 360.0; // degrees to inches
    previous_imu_reading_ = imu_.get_rotation() // Get ROTATION, not heading -- rotation is unbounded while heading is not
                            * M_PI / 180.0; // DEGREES (rotation is NOT reported in centidegrees!) to radians 
}

void odometry::update_position()
{
    // (1a) Get raw tracking wheel positions
    
    double current_parallel_tracker_pose = parallel_tracker_.get_position() / 100.0 // centidegrees to degrees
                                           * tracking_wheel_circumference_ / 360.0; // degrees to inches
    double current_perpendicular_tracker_pose = perpendicular_tracker_.get_position() / 100.0 // centidegrees to degrees
                                                * tracking_wheel_circumference_ / 360.0; // degrees to inches
    double current_imu_reading_ = imu_.get_rotation() * M_PI / 180.0; // degrees to radians

    // (1b) Convert to deltas

    double delta_parallel_tracker = current_parallel_tracker_pose - previous_parallel_tracker_pose_;
    double delta_perpendicular_tracker = current_perpendicular_tracker_pose - previous_perpendicular_tracker_pose_;
    double delta_imu_reading = current_imu_reading_ - previous_imu_reading_;

    // (2a) Remove rotation terms

    double center_arc_parallel = delta_parallel_tracker + parallel_tracker_offset_ * delta_imu_reading;
    double center_arc_perpendicular = delta_perpendicular_tracker + perpendicular_tracker_offset_ * delta_imu_reading;

    // (2b) Handle local movement (arcs and trigonometry)

    double center_chord_parallel;
    double center_chord_perpendicular;

    if (fabs(delta_imu_reading) >= 0.001) // Not sure about how small the angle needs to be for division by zero to start mattering
    {
        center_chord_parallel = center_arc_parallel / delta_imu_reading // radius of arc movement
                                * 2.0 * sin(delta_imu_reading / 2.0);
        center_chord_perpendicular = center_arc_perpendicular / delta_imu_reading // radius of arc movement
                                     * 2.0 * sin(delta_imu_reading / 2.0);
    }
    else
    {
        center_chord_parallel = center_arc_parallel;
        center_chord_perpendicular = center_arc_perpendicular;
    }

    // (3) Translate local movement to global movement (matrix multiplication, then add the deltas to the pose)

    double theta_mid = theta_ + delta_imu_reading / 2.0; // The direction the chord is pointing (the direction of the robot halfway through the arc)

    x_ += center_chord_parallel * sin(theta_mid) + center_chord_perpendicular * cos(theta_mid);
    y_ += center_chord_parallel * cos(theta_mid) - center_chord_perpendicular * sin(theta_mid);
    theta_ += delta_imu_reading;

    // (4) Set previous pose values

    previous_parallel_tracker_pose_ = current_parallel_tracker_pose;
    previous_perpendicular_tracker_pose_ = current_perpendicular_tracker_pose;
    previous_imu_reading_ = current_imu_reading_;
}
