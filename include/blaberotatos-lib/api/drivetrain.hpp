#include "main.h"

#include "pros/motors.hpp"

class tank_drivetrain
{
    private: // Motor groups should be hidden at a lower level of abstraction
        pros::MotorGroup left_motor_group;
	    pros::MotorGroup right_motor_group;
    
    public:
        tank_drivetrain();

        struct tank_drive_data_struct 
        {
            double left;
            double right;
        };

        int move(tank_drive_data_struct tankDriveData);

        tank_drive_data_struct arcade(double speed, double turn);
};