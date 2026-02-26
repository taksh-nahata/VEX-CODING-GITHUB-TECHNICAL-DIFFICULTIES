#include "main.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Announce Autons
void LEFT_SIDE_AWP();
void RIGHT_SIDE_ROUTE(); // Make sure this name matches autons.cpp
void auton_skills();
void LEFT_SIDE_NO_AWP();
void auton_button_5();
void auton_button_6();
void auton_button_8();
void default_constants(); 

// ----------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// ----------------------------------------------------------------------------

ez::Drive chassis(
    {-8, -9, 10},    // Left Ports
    {18, 19, -20},   // Right Ports
    17,              // IMU
    3.25,            // Wheel Diameter
    600,             // Cartridge RPM
    2                // Gear Ratio
);

// PROS 4 Syntax
pros::Motor intake(-15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);      
pros::Motor hood_motor(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 

// DISTANCE SENSOR (Added based on previous context)
pros::Distance dist_sensor(14); 

// PNEUMATICS
pros::ADIDigitalOut matchload_piston('A');
pros::ADIDigitalOut right_descore_piston('B');
pros::ADIDigitalOut middle_goal_piston('C');
pros::ADIDigitalOut hood_piston('D');
pros::ADIDigitalOut aligner_piston('E'); // NEW ALIGNER

// TOGGLE FLAGS
bool matchloadOn = false;
bool rightDescoreOn = false;
bool middleGoalOn = false;
bool topOutakeOn = false;
bool bottomIntakeOn = false;
bool alignerOn = false; // NEW FLAG

// ----------------------------------------------------------------------------
// INITIALIZATION
// ----------------------------------------------------------------------------
void initialize() {
  ez::ez_template_print();
  pros::delay(500); 

  // INSANE CONTROL SETTINGS
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  
  // PID TUNING
  default_constants(); 

  // AUTONOMOUS SELECTOR
  ez::as::auton_selector.autons_add({
      {"Button 1\n\n(L1) Left Side Route AWP", LEFT_SIDE_AWP},
      {"Button 2\n\n(R1) Right Side Route", RIGHT_SIDE_ROUTE},
      {"SKILLS\n\nFull Skills Routine", auton_skills},
      {"Button 4\n\n(L2) Left Side Route No AWP", LEFT_SIDE_NO_AWP},
      {"Button 5\n\nSKILLS JUST PARK", auton_button_5},
      {"Button 6\n\nTurn 360 Degrees", auton_button_6},
      {"Button 8\n\nEmpty slot", auton_button_8},
  });

  chassis.initialize();
  ez::as::initialize();
}

void disabled() {}
void competition_initialize() {}

// ----------------------------------------------------------------------------
// AUTONOMOUS RUNNER
// ----------------------------------------------------------------------------
void autonomous() {
  chassis.pid_targets_reset();       
  chassis.drive_sensor_reset();      
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 
  ez::as::auton_selector.selected_auton_call(); 
}

// ----------------------------------------------------------------------------
// BUTTON LOGIC
// ----------------------------------------------------------------------------

void rightDescoreD() {
    if (!rightDescoreOn) {
        right_descore_piston.set_value(true);
        rightDescoreOn = true;
        hood_motor.move(0);   
        hood_piston.set_value(false);
        topOutakeOn = false;
    } else {
        right_descore_piston.set_value(false);
        rightDescoreOn = false;
    }
}

void middleGoalD() {
    if (!middleGoalOn) {
        middle_goal_piston.set_value(true);
        hood_piston.set_value(false);
        hood_motor.move(-127); 
        intake.move(-127);     
        middleGoalOn = true;
        
        // Reset Conflicts
        topOutakeOn = false;
        bottomIntakeOn = false;
        // Don't necessarily need to reset matchload/aligner here unless they physically collide
    } else {
        middle_goal_piston.set_value(false);
        middleGoalOn = false;
    }
}

// --- NEW ALIGNER LOGIC ---
// Mapped to Button Y
void alignerD() {
    if (!alignerOn) {
        matchload_piston.set_value(false);
        matchloadOn = false;
        pros::delay(250);
        aligner_piston.set_value(true);
        alignerOn = true;

        // FORCE MATCHLOAD OFF (Mutual Exclusion)
        
    } else {
        aligner_piston.set_value(false);
        alignerOn = false;
    }
}

// --- UPDATED MATCHLOAD LOGIC ---
void matchloadD() {
    if (!matchloadOn) {
        // FORCE ALIGNER OFF (Mutual Exclusion)
        aligner_piston.set_value(false);
        alignerOn = false;
        pros::delay(250);
        matchload_piston.set_value(true);
        matchloadOn = true;

        

        // Turn off other stuff
        middle_goal_piston.set_value(false);
        middleGoalOn = false;
        topOutakeOn = false;
        bottomIntakeOn = true;
        
        intake.move(-127);    
        hood_piston.set_value(false);
        hood_motor.move(0);   
    } else {
        matchload_piston.set_value(false);
        matchloadOn = false;
        pros::delay(250);
        aligner_piston.set_value(true);
        alignerOn = true;
    }
}

void bottomIntakeD() {
    if (!bottomIntakeOn) {
        intake.move(-127);    
        bottomIntakeOn = true;
    } else {
        intake.move(127);    
        bottomIntakeOn = false;
    }
}

void topOutakeD() {
    if (!topOutakeOn) {
        hood_motor.move(-127); 
        hood_piston.set_value(true);
        middle_goal_piston.set_value(false);
        matchload_piston.set_value(false);
        aligner_piston.set_value(true); // Safety: retract aligner when shooting
        alignerOn = true;
        right_descore_piston.set_value(false);
        rightDescoreOn = false;

        intake.move(-127);    
        topOutakeOn = true;
        
        middleGoalOn = false;
        matchloadOn = false;
        bottomIntakeOn = true;
    } else {
        hood_motor.move(0);   
        hood_piston.set_value(false);
        topOutakeOn = false;
    }
}

void stopIntake() {
    intake.move(0);
    hood_motor.move(0);
    hood_piston.set_value(false);
    bottomIntakeOn = false;
    topOutakeOn = false;
    
    // Optional: Reset pistons to safe state?
    // matchload_piston.set_value(false);
    // aligner_piston.set_value(false);
}

// ----------------------------------------------------------------------------
// DRIVER CONTROL
// ----------------------------------------------------------------------------
void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST); 

  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT); 

    // TRIGGER AUTON (B + DOWN)
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        autonomous();
    }

    // BUTTONS
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) rightDescoreD();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) middleGoalD();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) matchloadD();
    
    // Mapped Aligner to 'L2'
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) alignerD();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) bottomIntakeD();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) topOutakeD();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) stopIntake();

    pros::delay(ez::util::DELAY_TIME);
  }
}