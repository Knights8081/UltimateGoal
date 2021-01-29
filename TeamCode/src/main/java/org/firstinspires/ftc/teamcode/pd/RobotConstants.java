package org.firstinspires.ftc.teamcode.pd;

/**
 * This in Not and opmode
 * List of constants for the robot.
 * Contains Arm, Servo, and Idol Hand Constants.
 *
 * Pull by (RobotConstants.NAME)
 *
 * @author Luke Frazer
 */
public class RobotConstants {

    /* Servo Constants */
    public static final double CLAW_MIN_RANGE       =   0.05;
    public static final double CLAW_MAX_RANGE       =   1.3;
    public static final double CLAW_HOME            =   0.08;
    public static final double CLAW_SPEED           =   0.05;

    /* Idol Hand Constants */
    public static final double IDOLHAND_MIN_RANGE   =   0.05;
    public static final double IDOLHAND_MAX_RANGE   =   1.3;

    /* Autonomous Constants */
    public static final double DRIVE_GEAR_REDUCTION    = .8;         // was .9375
                                                                    // This is < 1.0 if geared UP (was .667) after it was .625, after that it was .475, then .45
                                                                    //  but it changed again because it was going too far
                                                                    // WAS 1 for this year but trying .8
    public static final double WHEEL_DIAMETER_INCHES   = 1.96 ;     // For figuring circumference (THIS IS FOR ODO WHEELS, MECAMUM ARE 4.0
    public static final double COUNTS_PER_MOTOR_REV    = 560 ;      //560 is HD Hex 20:1, 1120 is HD Hex 40:1
//    public static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double COUNTS_PER_INCH         = (800 / 1.96 * Math.PI);
        //(408.16 / Math.PI); // odometry wheels are 800 ticks per rotation, and the diameter is 1.96in
                                                                 // so 800 / 1.96 is 408.16 ticks per inch
    public static final double INCHES_PER_COUNT        = 0.00078027;
    public static final double FAST_DRIVE              = 0.95;
    public static final double FAST_TURN               = 0.5;
    public static final double STRAFE_SPEED            = 0.5;
    public static final double STRAFE_CONSTANT         = .85;


    public static final double BOOK_IT                 = 0.85;
    public static final double ABOVE_SPEED             = 0.75;
    public static final double DRIVE_SPEED             = 0.65;
    public static final double TURN_SPEED              = 0.55;
    public static final double MEDIUM_SPEED            = 0.45;
    public static final double SLOW_SPEED              = 0.3;
    public static final double CREEP_SPEED             = 0.2;
    public static final double REAL_SLOW               = 0.1;










}
