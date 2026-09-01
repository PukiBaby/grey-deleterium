#include "main.h"

#include "blaberotatos-lib/api/drivetrain.hpp"

#include "ports.hpp"

#include <atomic>

// Setup for the macro (I kept this one outside because if I make a macro class, it should not be involved with the drivetrain)
std::atomic<bool> macro_running{false};

// Setup for the drivetrain

pros::Controller master(pros::E_CONTROLLER_MASTER);
tank_drivetrain drivetrain({PORT_L1, PORT_L2, PORT_L3}, {PORT_R1, PORT_R2, PORT_R3});

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() 
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() 
{}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() 
{}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{}

void macro_routine()
{
	drivetrain.spin(-0.25, 2000)
			  .spin(0.25, 2000); // Testing motion chaining
}

void macro_worker()
{
	while (true) // Makes the macro_task reusable
	{
		// Using cooperative cancellation rather than just killing the task
		pros::Task::notify_take(true, // Zeros the notification counter when the task wakes up
								TIMEOUT_MAX);
		drivetrain.drive({0, 0});
		macro_routine();
		macro_running = false;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() 
{
	static pros::Task macro_task(macro_worker);
	while (true) 
	{
		if (macro_running)
		{
			if (master.get_digital(DIGITAL_B)) 
			{
				drivetrain.abort_requested = true;
				pros::lcd::print(0, "Abort requested");
			}
		}
		else
		{
			if (master.get_digital(DIGITAL_A))
			{
				drivetrain.abort_requested = false;
				macro_running  = true;
				macro_task.notify();
				continue; // Skip the rest of the drivetrain code and go to the while loop's beginning
			}

			auto // Tells the compiler to infer the type from the initializer (identical to tank_drivetrain::tank_drive_data_struct)
			inputs = tank_drivetrain::arcade(master.get_analog(ANALOG_LEFT_Y)/127.0, // Note that in C++, doubles that have the same value as integers must be written .0 to indicate that we are doing operations with a double 
										     master.get_analog(ANALOG_RIGHT_X)/127.0); // Normalizing to the range from -1.0 to 1.0
			drivetrain.drive(inputs); 
			pros::delay(20);
		}
	}
}
