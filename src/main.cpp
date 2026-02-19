#include "main.h"
#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/misc.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"   // IWYU pragma: keep
#include "pros/optical.hpp"     // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep

#include "RclTracking.hpp"

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
inline pros::MotorGroup leftMotors({1, -2, 3}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({-4, 5, -6}, pros::MotorGearset::blue);

inline pros::Imu imu(11);

inline lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.375,
                              3.25,
                              450,
                              2
);

inline lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
inline lemlib::ControllerSettings lateral_controller(
                                              0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
inline lemlib::ControllerSettings angular_controller(0, // proportional gain (kP)
                                            0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Chassis
inline lemlib::Chassis chassis( drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Distance
inline pros::Distance back_dist(7);
inline pros::Distance right_dist(8);
inline pros::Distance front_dist(9);
inline pros::Distance left_dist(10);

// Rcl setup
inline RclSensor front_rcl(&front_dist, -1.0, 3.2, 0.0, 15.0);
inline RclSensor right_rcl(&right_dist, 4.5, 0.0, 90.0, 15.0);
inline RclSensor back_rcl(&back_dist, 5.375, -4.25, 180.0, 15.0);
inline RclSensor left_rcl(&left_dist, 4.5, 0.0, 270.0, 15.0);
inline RclTracking RclMain(&chassis, 30, true, 0.5, 4.0, 10.0, 6.0, 20);

// loaders
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 3);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 3);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 3);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 3);

// legs
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);

// Disable Line
inline Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);
inline Circle_Obstacle centerGoals(0, 0, 5);

void initialize() {
	pros::lcd::initialize();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::delay(20);
	}
}