#include "main.h"

#include "pros/motors.hpp"

#include "blaberotatos-lib/api/drivetrain.hpp"

#include "ports.hpp"

#include <cmath>
#include <algorithm>

// Constructor using a member initializer list
tank_drivetrain::tank_drivetrain()
    : left_motor_group({PORT_L1, PORT_L2, PORT_L3}),
        right_motor_group({PORT_R1, PORT_R2, PORT_R3})
{}

int tank_drivetrain::move(tank_drive_data_struct tank_drive_data) // Takes a normalized input from -1.0 to 1.0; also, used move_voltage, which does not respect brake modes
{
    left_motor_group.move_voltage(tank_drive_data.left * 12000.0);
    right_motor_group.move_voltage(tank_drive_data.right * 12000.0);

    return 0;
}

tank_drivetrain::tank_drive_data_struct 
tank_drivetrain::arcade(double speed, double turn) // Positive value of turn = go right = left side should be faster; hence, add turn to the left side 
{
    tank_drivetrain::tank_drive_data_struct arcade_drive_data;

    double normalization = std::max( // std::max(..., 1.0) is what stops this from scaling up small values
        std::abs(speed) + std::abs(turn), // Mathematically, working through the absolute value cases, abs(speed) + abs(turn) gives max(abs(speed + turn), abs(speed - turn)), which is what we want to scale to (scale to -1.0 to 1.0)
        1.0
    ); 

    arcade_drive_data.left = (speed + turn) / normalization;
    arcade_drive_data.right = (speed - turn) / normalization;

    return arcade_drive_data;
}