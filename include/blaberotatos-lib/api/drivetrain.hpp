#include "main.h"

#include "pros/motors.hpp"

#include <vector>
#include <cstdint>

class tank_drivetrain
{
    private: // Motor groups should be hidden at a lower level of abstraction
        pros::MotorGroup left_motor_group;
	    pros::MotorGroup right_motor_group;
    
    public:
        tank_drivetrain(std::vector<std::int8_t> left_ports, // {PORT_L1, PORT_L2, PORT_L3}
                        std::vector<std::int8_t> right_ports); // {PORT_R1, PORT_R2, PORT_R3}

        struct tank_drive_data_struct 
        {
            double left;
            double right;
        };

        int drive(tank_drive_data_struct tank_drive_data);

        tank_drive_data_struct arcade(double speed, double turn);
};