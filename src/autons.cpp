#include "main.h"

/////
// EXTERN DECLARATIONS
/////
extern pros::Motor intake;
extern pros::Motor hood_motor;
extern pros::ADIDigitalOut matchload_piston;
extern pros::ADIDigitalOut right_descore_piston;
extern pros::ADIDigitalOut middle_goal_piston;
extern pros::ADIDigitalOut hood_piston;
extern pros::ADIDigitalOut aligner_piston;

// DISTANCE SENSOR (Port 10 in main.cpp)
extern pros::Distance dist_sensor; 

/////
// HELPER FUNCTIONS
/////

// --- NEW: SENSOR CORRECTION FUNCTION ---
// Adjusts the robot to be exactly 'target_in' inches away from the goal
// Sensor on BACK: 
// - If Reading > Target (Too Far) -> Drive Backward (-)
// - If Reading < Target (Too Close) -> Drive Forward (+)
void correct_to_goal(double target_in, int timeout_ms) {
  int start_time = pros::millis();
  double target_mm = target_in * 25.4;
  
  while (pros::millis() - start_time < timeout_ms) {
    int current_dist = dist_sensor.get(); // Get distance in mm

    // Filter out garbage readings
    if (current_dist < 10 || current_dist > 2000) break; 

    int error = current_dist - target_mm;

    // Tolerance: Stop if within .75 inch (12mm)
    if (abs(error) < 15) break;

    // P-Control: 
    // Error + (Too Far) -> Speed - (Backwards)
    int speed = error * -1.5; 

    // Cap speed
    if (speed > 60) speed = 60;
    if (speed < -60) speed = -60;

    chassis.drive_set(speed, speed);
    pros::delay(20);
  }
  chassis.drive_set(0, 0);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

void jiggle(int time_ms) {
  int start_time = pros::millis();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  while (pros::millis() - start_time < time_ms) {
    chassis.drive_set(45, 45); 
    pros::delay(250);
    chassis.drive_set(0,0);
    pros::delay(100);
    chassis.drive_set(-45, -45);
    pros::delay(100);
  }
  chassis.drive_set(0, 0);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
}

void set_intake(int speed) { intake.move(speed); }
void set_hood_motor(int speed) { hood_motor.move(speed); }

void bottom_intake() {
  set_intake(-127);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void top_outtake() {
  set_hood_motor(-127);
  hood_piston.set_value(true);
  matchload_piston.set_value(false);
  middle_goal_piston.set_value(false);
  set_intake(-127); 
}

void stop_intake() {
  set_intake(0);
  set_hood_motor(0);
  hood_piston.set_value(false);
}
void alignerDown() {
  aligner_piston.set_value(true);
}
void alignerUp() {
  aligner_piston.set_value(false);
}

void top_intake() {
  set_hood_motor(0);
  hood_piston.set_value(false);
}

void bottom_outtake() { set_intake(127); }

void middle_goal_action() {
  middle_goal_piston.set_value(true);
  hood_piston.set_value(false);
  set_hood_motor(-127);
  set_intake(-127);
}

/////
// CONSTANTS
/////
const int DRIVE_SPEED = 110;
const int TURN_SPEED  = 100;
const int SWING_SPEED = 90;

void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.slew_turn_constants_set(5_deg, 50); 
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.pid_turn_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);
}

// ============================================================================
// OPTIMIZED AUTONS
// ============================================================================

// BUTTON 1: LEFT SIDE "AWP"
void LEFT_SIDE_AWP() {
  chassis.pid_drive_set(10_in, 110);
  chassis.pid_wait_quick_chain();   

  chassis.pid_turn_relative_set(-60_deg, TURN_SPEED);
  chassis.pid_wait(); 

  bottom_intake();
  top_intake();
  pros::delay(100);
  chassis.pid_drive_set(21_in, 110);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_relative_set(-75_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(26.25_in, 110);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_relative_set(-42.5_deg, TURN_SPEED);
  chassis.pid_wait();

  // DRIVE TO GOAL + CORRECTION
  chassis.pid_drive_set(-9_in, 110); 
  chassis.pid_wait();
  
  // Ensure we are exactly 4 inches from the goal before shooting
  correct_to_goal(4.0, 1000); 

  top_outtake();
  pros::delay(1250); 

  matchload_piston.set_value(true);
  bottom_intake();
  top_intake();
  chassis.pid_turn_relative_set(-6_deg, TURN_SPEED);

  chassis.pid_drive_set(28.5_in, 110);
  chassis.pid_wait_quick_chain(); 
  pros::delay(450); 

  chassis.pid_drive_set(-14_in, 110);
  chassis.pid_wait();

  matchload_piston.set_value(false);

  chassis.pid_turn_relative_set(49.5_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-48.5_in, 110);
  chassis.pid_wait();

  middle_goal_action();
  pros::delay(1000);
  middle_goal_piston.set_value(false);
  top_intake();
  chassis.pid_drive_set(14_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-136_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(43_in, 110);
  chassis.pid_wait();
  stop_intake();
  chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11.5_in, 110);
  chassis.pid_wait();
  bottom_outtake();
}

// BUTTON 2: RIGHT SIDE ROUTE (Mirrored)
void RIGHT_SIDE_ROUTE() {
  // RIGHT SIDE REVERSED CODE
chassis.pid_drive_set(10_in, 100);
chassis.pid_wait_quick_chain();   

// Inverted: -60 -> 60
chassis.pid_turn_relative_set(60_deg, TURN_SPEED);
chassis.pid_wait(); 

bottom_intake();
top_intake();


chassis.pid_drive_set(22_in, 100);
chassis.pid_wait_quick_chain(); 
pros::delay(100);

// Inverted: -85 -> 85
chassis.pid_turn_relative_set(85_deg, TURN_SPEED);
chassis.pid_wait();

chassis.pid_drive_set(25_in, 110);
chassis.pid_wait_quick_chain(); 

// Inverted: -41.5 -> 41.5
chassis.pid_turn_relative_set(41.5_deg, TURN_SPEED);
chassis.pid_wait();

alignerDown(); // Deploy aligner

// SENSOR CORRECTION
chassis.pid_drive_set(-14_in, 100);
chassis.pid_wait();
correct_to_goal(4.0, 1000); // Sensor handles the precision

top_outtake();
pros::delay(1250); 
chassis.pid_turn_relative_set(-4_deg, TURN_SPEED); // Nudge turn to align with goal
chassis.pid_wait_quick_chain();
matchload_piston.set_value(true);
bottom_intake();
top_intake();
alignerUp(); // Retract aligner

// Inverted: 4 -> -4

chassis.pid_drive_set(28_in, 90);
chassis.pid_wait_quick_chain(); 
pros::delay(150);

jiggle(500); // Unstuck/Grab balls

chassis.pid_drive_set(-7.5_in, 110);
chassis.pid_wait();
matchload_piston.set_value(false);

// Inverted: 50 -> -50
chassis.pid_turn_relative_set(135.5_deg, TURN_SPEED);
chassis.pid_wait();

chassis.pid_drive_set(48_in, 110);
chassis.pid_wait();

bottom_outtake();
}

// BUTTON 3: SKILLS (Single Global Frame)

void auton_skills() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  bottom_intake();
  chassis.pid_odom_set({{-7_in, 38_in, 0_deg}, fwd, 110});
  chassis.pid_wait_until({-4_in,33_in});
  matchload_piston.set_value(true);   
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  chassis.pid_odom_set({{-5_in,44_in}, rev, 100});
  chassis.pid_wait();
  middle_goal_action();
  pros::delay(1000);
  chassis.pid_odom_set({{{12_in, -24_in}, fwd, 100},
                        {{12_in, -12_in, 180_deg}, fwd, 100}});
  chassis.pid_wait_quick_chain();
    pros::delay(1000);
  /*chassis.pid_odom_set({{0_in, 12_in}, fwd, 100});
  chassis.pid_wait_quick_chain();
    pros::delay(1000);

  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
    pros::delay(1000);

  chassis.pid_odom_set({{0_in, 25_in}, fwd, 100});
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  chassis.pid_swing_set(ez::RIGHT_SWING, -135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  */
}

// BUTTON 4: Left side without AWP 
void LEFT_SIDE_NO_AWP() {
  chassis.pid_drive_set(10_in, 100);
  chassis.pid_wait_quick_chain();   

  chassis.pid_turn_relative_set(-60_deg, TURN_SPEED);
  chassis.pid_wait(); 

  bottom_intake();
  top_intake();
  pros::delay(100);
  chassis.pid_drive_set(21_in, 100);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_relative_set(-85_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_relative_set(-40.5_deg, TURN_SPEED);
  chassis.pid_wait();
  alignerDown();
  // SENSOR CORRECTION
  chassis.pid_drive_set(-14_in, 100);
  chassis.pid_wait();
  //correct_to_goal(4.0, 1000);

  top_outtake();
  pros::delay(1250); 

  matchload_piston.set_value(true);
  bottom_intake();
  top_intake();
  alignerUp();
  chassis.pid_turn_relative_set(4_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(28_in, 80);
  chassis.pid_wait_quick_chain();
  pros::delay(150);
  jiggle(600); 

  chassis.pid_drive_set(-13_in, 110);
  chassis.pid_wait();
  matchload_piston.set_value(false);


  chassis.pid_turn_relative_set(52_deg, TURN_SPEED);

  chassis.pid_wait();



  chassis.pid_drive_set(-47.5_in, 110);

  chassis.pid_wait();



  middle_goal_action();
  pros::delay(500);
  bottom_outtake();
  pros::delay(250);
  middle_goal_action();


}

// BUTTON 5: SKILLS JUST PARK
void auton_button_5() {
  chassis.pid_drive_set(-9.25_in,100);
  chassis.pid_wait();
  matchload_piston.set_value(true);
  bottom_intake();
  chassis.pid_drive_set(10_in,75);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in,127);
  chassis.pid_wait();
  pros::delay(4000);
  matchload_piston.set_value(false);
  chassis.pid_drive_set(6_in,85);
  chassis.pid_wait();
}

void auton_button_6() {
  // Initialize odometry
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // Drive off start line
  chassis.pid_odom_set(42.5_in, 100);
  chassis.pid_wait();

  // Turn left 90 deg
  chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  matchload_piston.set_value(true);
  bottom_intake();
  pros::delay(300);

  //matchload
  chassis.pid_drive_set(13_in, 80);
  chassis.pid_wait();
  pros::delay(1200);
  //chassis.pid_drive_set(4_in, 50);
  //chassis.pid_wait();


  // Back out from matchload
  chassis.pid_odom_set(-15_in, 80);
  chassis.pid_wait();
//prepare to cross field 1
  chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(20_in, 100);
  chassis.pid_wait();
  alignerUp();
  chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  matchload_piston.set_value(true);
  
  //cross field 1
  chassis.pid_odom_set(55_in, 100);
  chassis.pid_wait();
//prepare to score 1
  chassis.pid_turn_relative_set(-40_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(26.5_in,100);
  chassis.pid_wait();
  matchload_piston.set_value(false);
  chassis.pid_turn_relative_set(40_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //score on goal 1
  alignerDown();
  chassis.pid_odom_set(-13_in,100);
  chassis.pid_wait();
  correct_to_goal(3.5, 1000);
  top_outtake();
  bottom_intake();
  pros::delay(2250);

  //matchload 2
  alignerUp();
  pros::delay(250);
  matchload_piston.set_value(true);
  top_intake();
  chassis.pid_odom_set(29_in, 85);
  chassis.pid_wait();
  pros::delay(1800);
  //score on goal 2
  alignerDown();
  chassis.pid_odom_set(-29_in, 85);
  chassis.pid_wait();
  top_outtake();
  bottom_intake();
  pros::delay(2500);
  hood_piston.set_value(false);
  pros::delay(700);

 //prepare to cross field
  top_outtake();
  alignerUp();
  chassis.pid_odom_set(15_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  matchload_piston.set_value(true);
  //cross field 2
  chassis.pid_odom_set(96_in, 100);
  chassis.pid_wait();

  //matchload 3
  top_intake();
  chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(17.5_in, 85);
  chassis.pid_wait();
  pros::delay(1650);
  
  //prepare to cross field 3
  chassis.pid_odom_set(-15_in, 100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(20_in, 100);
  chassis.pid_wait();
  alignerUp();
  chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  matchload_piston.set_value(true);
  
  //cross field 3
  pros::delay(100);
  chassis.pid_odom_set(55_in, 100);
  chassis.pid_wait();

  //prepare to score 3
  chassis.pid_turn_relative_set(-40_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(33.5_in,100);
  chassis.pid_wait();
  matchload_piston.set_value(false);
  chassis.pid_turn_relative_set(40_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //score on goal 3
  alignerDown();
  chassis.pid_odom_set(-13_in,100);
  chassis.pid_wait();
  correct_to_goal(3.5, 1000);
  top_outtake();
  bottom_intake();
  pros::delay(2250);

  //matchload 4
  alignerUp();
  pros::delay(250);
  matchload_piston.set_value(true);
  top_intake();
  chassis.pid_odom_set(29_in, 85);
  chassis.pid_wait();
  pros::delay(1650);
  //score on goal 4
  alignerDown();
  chassis.pid_odom_set(-29_in, 85);
  chassis.pid_wait();
  top_outtake();
  bottom_intake();
  pros::delay(2500);
  hood_piston.set_value(false);

  //prepare to park
  chassis.pid_odom_set(15_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(24_in, 85);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  double drive_dist = (-dist_sensor.get() / 25.4) + 3.0;
  alignerUp();
  chassis.pid_odom_set(drive_dist, 85);
  chassis.pid_wait();
  //chassis.pid_odom_set(15_in, 85);
  //chassis.pid_wait();
  chassis.pid_turn_relative_set(-95_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //park
  bottom_outtake();
  chassis.pid_odom_set(10_in, 100);
  chassis.pid_wait(); 
  chassis.pid_odom_set(24_in, 80);
  chassis.pid_wait();

}

void auton_button_8() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_odom_set(15_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(24_in, 85);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  alignerDown();
  pros::delay(250);
  double drive_dist = (-dist_sensor.get() / 25.4) + 3.0;
  pros::delay(250);
  alignerUp();
  chassis.pid_odom_set(drive_dist, 105);
  chassis.pid_wait();
  //chassis.pid_odom_set(15_in, 85);
  //chassis.pid_wait();
  chassis.pid_turn_relative_set(-95_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //park
  bottom_outtake();
  chassis.pid_odom_set(10_in, 100);
  chassis.pid_wait(); 
  chassis.pid_odom_set(26_in, 80);
  chassis.pid_wait();
  pros::delay(500);
  bottom_intake();
  chassis.CURRENT_BRAKE = pros::E_MOTOR_BRAKE_HOLD;
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}