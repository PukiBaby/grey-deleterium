#include "main.h"

#include "pros/motors.hpp"

#include "blaberotatos-lib/api/drivetrain.hpp"

#include "ports.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdint>

// Constructor using a member initializer list
// I cannot just use this https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Named_Parameter for the motor groups because PROS motor groups have no default constructors (they cannot be initialized and then be fed the values later)
tank_drivetrain::tank_drivetrain(std::vector<std::int8_t> left_ports, // {PORT_L1, PORT_L2, PORT_L3}
                                 std::vector<std::int8_t> right_ports) // {PORT_R1, PORT_R2, PORT_R3}
    : left_motor_group(left_ports),
      right_motor_group(right_ports)
{}

int tank_drivetrain::drive(tank_drive_data_struct tank_drive_data) // Takes a normalized input from -1.0 to 1.0; also, used move_voltage, which does not respect brake modes
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

tank_drivetrain& tank_drivetrain::spin(double intensity, double timeout_ms)
{
    std::uint32_t start = pros::millis();
    while (pros::millis() - start < timeout_ms)
    {
        drive(arcade(0.0, intensity));
        pros::delay(20);
    }
    return *this;
}