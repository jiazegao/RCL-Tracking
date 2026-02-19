# RCL Tracking

## File Setup
1. Copy and paste "RclTracking.hpp" into the "include" folder in the PROS project
2. Copy and paste "RclTracking.cpp" into the "src" folder in the PROS project
2. Include the file in "main.cpp" in the "src" folder ( #include "RclTracking.hpp" )

## Object Declarations

### RclTracking
1. The constructor takes 1 mandatory argument: A pointer to a lemlib::Chassis object.
2. The constructor takes multiple optional arguments:
- "int frequency" - Records the amount of refreshes per second; default is 20hz.
- "bool autoSync" - If set to true, enables automatic sync to lemlib::chassis; default is true.
- "double minDelta" - The minimum allowed difference in the calculated x / y coordinate of a sensor and the lemlib bot position for the calculation result to be recognized; this can be used to account for sensor reading variations; default is 1.2 inches.
- "double maxDelta" - The maximum allowed difference in the calculated x / y coordinate of a sensor and the lemlib bot position for the calculation result to be recognized; this can be used to account for objects on the field; default is 3.0 inches.

> e.g.
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors );
> RclTracking RclMain ( &chassis );

### RclSensor
1. The constructor takes 4 arguments:
- A pointer to an pros::Distance object
- Horizontal offset of the sensor relative to the tracking center (in inches; right -> positive; left -> negative)
- Vertical offset of the sensor relative to the tracking center (in inches; forward -> positive; backward -> negative)
- The heading of the sensor (Forward -> 0; Right -> 90; Backward -> 180; Left -> 270)

> e.g. 	pros::Distance distance (2); RclSensor sensor1 (	&distance, 3.0, -2.9, 90);		-> A sensor 3.0 inches to the right and 2.9 inches to the back of the tracking center; facing right

### Circle Obstacle Declarations
1. The constructer takes 3 arguments:
- The x coordinate of the origin of the obstacle
- The y coordinate of the origin of the obstacle
- The radius of the obstacle

> e.g.	obstacle obstacle1 ( 12.8, -32.1, 4.0 );	-> An obstacle located at (12.8, -32.1) with a radius of 4.0 inches.

## How to Use

1. For Rcl positioning to be available at all times during a match, call the function "RclMain.startTracking()" inside "void initialize()." This function will start the main loop for RclMain, and the position of the bot will be consistently calculated based on Rcl sensors in the background. (It will also be automatically synced to the lemlib::Chassis object if autoSync is set to True.)

2. To update the position of the bot based on Rcl sensors, simply call "RclMain.updateBotPose()." This function will automatically set the position of chassis based on Rcl sensor readings. However, note that:
- this function should only be called after "RclMain.startTracking()" has been called;
- there shouldn't be any need to call this function if autoSync is enabled.

3. To perform standard distance reset, call "RclMain.updateBotPose(&RclSensor)," where "&RclSensor" can be any of the declared RclSensor objects. However, note that:
  - all filters, except for invalid sensor readings, will be disabled for the reset;
  - only use this feature if you know for sure that nothing will be in the way of the distance sensor.

4. To get the current position of the bot calculated by RclMain, use the method "RclMain.getRclPose()." (If autoSync is enabled, this should be close to, if not the exact same as, the position of lemlib::chassis.)

5. Sensor values will vary even if the bot is not moving at all. If you know for sure that the bot is not moving and want higher accuracy, call "RclMain.startAccumulating()". Once called, the values of each sensor will start to accumulate in the background. To stop the accumulation, call "RclMain.stopAccumulating ()". Once called, the sensor values will stop accumulating, and the average reading of each sensor will be will proceed to position calculations. However, note that:
- minDelta detection will be temporarily disabled during this process;
- even if autoSync is disabled, the position of the bot in lemlib will still be synced after the position of the bot has been calculated based on the average sensor readings.

6. If you're resetting the position of the lemlib chassis, make sure you also call "RclMain.setRclPose( lemlib::Pose newPosition )" to reset the position for RclMain.
