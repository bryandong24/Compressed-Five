#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller1(pros::E_CONTROLLER_MASTER);

//ADI Sensor Ports
#define DIGITAL_SENSOR_PORT_A 'A' 
#define DIGITAL_SENSOR_PORT_B 'B' 
#define DIGITAL_SENSOR_PORT_C 'C' 
#define DIGITAL_SENSOR_PORT_D 'D' 
#define DIGITAL_SENSOR_PORT_E 'E' 
#define DIGITAL_SENSOR_PORT_F 'F' 
#define DIGITAL_SENSOR_PORT_H 'H' 
#define DIGITAL_SENSOR_PORT_G 'G' 

// drive motors
//Motor Constructors
pros::Motor left_front_motor(20, pros::E_MOTOR_GEARSET_06, true); // port 11, blue gearbox, not reversed
pros::Motor left_middle_motor(7, pros::E_MOTOR_GEARSET_06, true); // port 12, blue gearbox, not reversed
pros::Motor left_back_motor(10, pros::E_MOTOR_GEARSET_06, false); // port 13, blue gearbox, reversed
pros::Motor right_front_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 14, blue gearbox, reversed
pros::Motor right_middle_motor(2, pros::E_MOTOR_GEARSET_06, false); // port 15, blue gearbox, reversed
pros::Motor right_back_motor(1, pros::E_MOTOR_GEARSET_06, true); // port 16, blue gearbox, not reversed
pros::Motor cata_motor(11, pros::E_MOTOR_GEARSET_36, true); //port 17, red gearbox, reversed
pros::Motor intake_motor(12, pros::E_MOTOR_GEARSET_06, false); 

// motor groups
pros::Motor_Group left_drive_group({left_front_motor, left_middle_motor,left_back_motor}); //all motors on the left side of the drive grouped
pros::Motor_Group right_drive_group({right_front_motor, right_middle_motor,right_back_motor}); //all motorsright side of the drive
pros::Motor_Group drive_total({left_front_motor, left_middle_motor,left_back_motor, right_front_motor, right_middle_motor,right_back_motor});

// Inertial Sensor on port 2
pros::Imu imu1(4); //imu port 4
pros::GPS gps1(18); //gps port 20

//Solenoids
pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT_F);
pros::ADIDigitalOut blocker (DIGITAL_SENSOR_PORT_G);
pros::ADIDigitalOut climb (DIGITAL_SENSOR_PORT_H);

//Preset Booleans for toggles
bool intake_bool (false);
bool cata_bool (false);
bool blocker_bool (false);
bool wings_bool (false);
bool climbs_bool (false);

//Encoders
pros::ADIEncoder encH('D', 'B', true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel H_tracker(&encH, lemlib::Omniwheel::NEW_275, -.1, 1);

// drivetrain settings
lemlib::Drivetrain drivetrain {
    &right_drive_group, // left drivetrain motors
    &left_drive_group, // right drivetrain motors
    11.5, // track width
    lemlib::Omniwheel::NEW_4, // wheel diameter
    300, // wheel rpm
	8 // chase power
};

// lateral motion controller
lemlib::ControllerSettings linearController(50, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            140, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             24.2, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             1.8 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &H_tracker, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

ASSET(sixOne_txt);
ASSET(sixOneB_txt)
ASSET(sixTwo_txt);
ASSET(sixThree_txt);
ASSET(sixFour_txt);
ASSET(sixFive_txt);
ASSET(sixSix_txt);
void autonomous() {

	chassis.setPose(39.804,-60.987,0);

	chassis.follow(sixOne_txt,15,4000); 
	chassis.waitUntil(31);
	intake_motor = 127; //Drop off matchload at side of goal 
	chassis.waitUntilDone();
	intake_motor = 0;

	chassis.turnTo(42.88, -41.445, 2500);

	chassis.follow(sixOneB_txt, 15, 4000);
	chassis.waitUntil(82); //At point right before elevation bar triball
	intake_motor = -127; //intake triball at elevation bar
	chassis.waitUntilDone();
	intake_motor = 0; //turning intake motor off since triball has been acquired under elevation bar

	chassis.follow(sixTwo_txt, 4, 6000, false);
	chassis.waitUntil(42);//Open wings here
	//Wing toggle code in the following two lines
	wings_bool = wings_bool == false;
	wings.set_value(wings_bool);
	//Triball is being whipped out of matchload zone
	chassis.waitUntil(50); //close wings here
	wings_bool = wings_bool == false;
	wings.set_value(wings_bool);

	chassis.follow(sixThree_txt, 4, 6000);
	chassis.waitUntil(4);
	wings_bool = wings_bool == false; //wing toggle
	wings.set_value(wings_bool);      //wing toggle
	intake_motor = 127; //outake triball  KEEP IT RUNNING OUT. I DO NOT WANT THE TRIBALL TO GET BACK INTO THE INTAKE
	chassis.waitUntilDone(); 
	wings_bool = wings_bool == false; //wing toggle
	wings.set_value(wings_bool);      //wing toggle
	intake_motor = 0;    //turn off intake

	chassis.follow(sixFour_txt,7, 4000, false);
	
	chassis.follow(sixFive_txt, 4, 5000);
	chassis.waitUntil(52);
	intake_motor = -127; //intake into bot
	chassis.waitUntil(65); 
	intake_motor = 0; //turn off intake motor during transport
	chassis.waitUntil(88);
	intake_motor = 127;  //dump triball
	chassis.waitUntilDone(); 
	intake_motor = 0; 

	chassis.turnTo(15, -17,3000); //we may need a turnTo function in between these blocks so the robot won't be super slow and turn like a snail since the headings are basically opposite of each other.

	chassis.follow(sixSix_txt,5, 5000);
	chassis.waitUntil(25);
	intake_motor = -127; //intake into the bot
	chassis.waitUntil(32);
	wings_bool = wings_bool == false; //wing toggle OPEN
	wings.set_value(wings_bool);      //wing toggle OPEN
	chassis.waitUntil(56);
	intake_motor = 127;  //Outake triball to get ready for scoring!
	chassis.waitUntilDone();
	intake_motor = 0;
	wings_bool = wings_bool == false; //wing toggle CLOSE
	wings.set_value(wings_bool);      //wing toggle CLOSE



    /**
    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has travelled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnTo(45, -45, 1000, true, 60);
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
    */
}

/**
 * Runs in driver control
 */

void opcontrol() {
    
	while (true) {

		int left = controller1.get_analog(ANALOG_LEFT_Y);
		int right = controller1.get_analog(ANALOG_RIGHT_Y);

		left_drive_group = left;
		right_drive_group = right;

		if (controller1.get_digital(DIGITAL_R1)) {
			cata_motor = 127;
		} else {
			cata_motor = 0;
		}
		
		if (controller1.get_digital_new_press(DIGITAL_R2)) {
			if(blocker_bool == false){
				blocker_bool = blocker_bool == false;
				blocker.set_value(blocker_bool);
				pros::delay(400);
			}
			climbs_bool = climbs_bool == false;
			climb.set_value(climbs_bool);

		pros::delay(20);
		}
		if (controller1.get_digital_new_press(DIGITAL_Y)) {
			wings_bool = wings_bool == false;
			wings.set_value(wings_bool);
		}
		if(controller1.get_digital(DIGITAL_L2)){
			intake_motor = -127;
		}else if(controller1.get_digital(DIGITAL_L1)){
			intake_motor = 127;
		}else{
			intake_motor = 0;
		}
		if(controller1.get_digital_new_press(DIGITAL_B)){
			blocker_bool = blocker_bool == false;
			blocker.set_value(blocker_bool);

		}
}
}