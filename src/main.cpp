#include "main.h"
#include "autons.hpp"
#include "disks.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

// motors
pros::Motor intake(15);
pros::Motor rightCata(20);
pros::Motor leftCata(14, 1);
//pistons
pros::ADIDigitalOut LPistons(3);
pros::ADIDigitalOut RPistons(2);
pros::ADIDigitalOut Boost(1);

//sensors
pros::Rotation cataRotation(21,1);


//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);



PID cataPID{0.7, 0, 1, 0, "Cata"};

// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1,-2,-3,-4}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{16,17,18,19}

  // IMU Port
  ,13

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.667

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 * xinghao is a big loser :((((()))))
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  //ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.


  cataPID.set_constants(0.1, 0, 0.4);
  cataRotation.reset_position();
  LPistons.set_value(0);
  RPistons.set_value(0);
  Boost.set_value(0);

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 10); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Red Match Auto\n\n Start against the roller.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  pros::lcd::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



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
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  //drive_example();
 // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
 
 //load cata 
 loadCata();
 pros::delay(1000);
 setCata(0);

 //rollers 

 chassis.set_drive_pid(18, 60);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_turn_pid(90, 50);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_drive_pid(7.5, 60);
 chassis.wait_drive();
 rollers(60, -100);
 pros::delay(1000);

 //going to shooting position 1

 chassis.set_drive_pid(-14.5, -40);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_turn_pid(100, 40);
 chassis.wait_drive();
 fireCata();
 pros::delay(1000);

//going to shooting position two 
 chassis.set_drive_pid(4, 50);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_turn_pid(220, 40);
 chassis.wait_drive();
 pros::delay(1000);
 loadCata();
 intake = 127; 
 chassis.set_drive_pid(39, 50);
 chassis.wait_drive();
 intake = 0; 
 pros::delay(1000);
 chassis.set_turn_pid(120, 40);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_drive_pid(8, 40);
 chassis.wait_drive();
 fireCata();


//shooting position three 
 chassis.set_drive_pid(-15.5, -40);
 chassis.wait_drive();
 pros::delay(1000);
 chassis.set_turn_pid(180, 40);
 chassis.wait_drive();
 loadCata();
 pros::delay(1000);
 intake = 127;
 chassis.set_drive_pid(60, 70);
 chassis.wait_drive();
 pros::delay(1000); 
 intake = 0; 
 Boost.set_value(1);
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

int timePressed = 0;

bool toggle;

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  leftCata.set_brake_mode(MOTOR_BRAKE_BRAKE);
  rightCata.set_brake_mode(MOTOR_BRAKE_BRAKE);
  bool LPistonState;
  bool RPistonState;
  bool boostState;
 
  while (true) {

    


    //chassis.tank(); // Tank control
    //chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade
    pros::lcd::print(1,"Tick Position: %ld \n", cataRotation.get_position());
    //Intake
    if(master.get_digital(DIGITAL_L1) == 1){
			intake = 127; 
		}
		else if(master.get_digital(DIGITAL_L2) == 1){
			intake = -127;
		}
		else{
			intake = 0;
		}
		
    //Cata
    
    if(master.get_digital(DIGITAL_R2)){
      setCata(127);
      timePressed = pros::c::millis();
    }
    else if(pros::c::millis() > timePressed + 500){
      setCata((7800-cataRotation.get_position())*.2);
      pros::delay(50);
      if(cataRotation.get_position() <= 7600){
         setCata(0);
      }
    }
    else {
      setCata(0);
    }
  
 

    //pistons 
    if(master.get_digital_new_press(DIGITAL_UP)){
      LPistonState = !LPistonState;
      LPistons.set_value(LPistonState);
    }
    if(master.get_digital_new_press(DIGITAL_RIGHT)){
      RPistonState = !RPistonState;
      RPistons.set_value(RPistonState);
    }    
    if(master.get_digital_new_press(DIGITAL_DOWN)){
      boostState = !boostState;
      Boost.set_value(boostState);
    }
 

    pros::lcd::print(6,"Cata angle: %d\n",cataRotation.get_angle());
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
  }

