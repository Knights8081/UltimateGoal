package org.firstinspires.ftc.teamcode.pd;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.nio.charset.CharsetDecoder;

//import org.firstinspires.ftc.teamcode.drivers.MaxbotixUltrasonicI2c;

public class HardwareSoftware {
    /* Variables for Odometry  ------------------------------------------------------------------*/
    Orientation angles;

    //TODO: Put in constans class
    static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    static final double oneRotationTicks = 800;
    static final double wheelRadius = 0.025; // in meters (0.05)
    static final double wheelDistanceApart = 0.345; // in meters (the right and left odo wheels are .167 m from the center one)
    static final double wheelDistanceApartInches = 13.583;
    private ElapsedTime timer = new ElapsedTime();

    private int leftEncoderPos = 0;
    private int rightEncoderPos = 0;
    private int centerEncoderPos = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;
    private double thetadeg = 0;
    private double cwheelradius = 0; //distance of center odometry wheel from center of robot
    private double LticksTelem = 0;
    private double RticksTelem = 0;
    private double CticksTelem = 0;
    private double DLeftTelem = 0;
    private double DRightTelem = 0;
    private double DCenterTelem = 0;
    private double currentPositionL = 0;
    private double currentPositionR = 0;
    private double previousPositionL = 0;
    private double previousPositionR = 0;
    private double previousTheta = 0;
    private double previousThataDeg = 0;
    private double deltaTheta = 0;


    /* Define Motor references ------------------------------------------------------------------*/
    private DcMotor  FLdrive    = null;
    private DcMotor  FRdrive    = null;
    private DcMotor  BLdrive    = null;
    private DcMotor  BRdrive    = null;
    private DcMotor  Intake     = null;
    private DcMotor  Delivery   = null;
    private DcMotor  Arm        = null;
    private DcMotor  LencoderDrive = null;
    private DcMotor  RencoderDrive = null;
    private DcMotor  CencoderDrive = null;

    /* Define Servo references ------------------------------------------------------------------*/
    private Servo deliveryServo     = null;
//    private CRServo armServo          = null;
    private Servo intakeServo     = null;
    private Servo armStableServo  = null;

//    private DigitalChannel magnetLimitSwitch = null;
//    private DistanceSensor distanceSensor = null;

    boolean ArmDown= false;

//    private VoltageSensor vs = null;
    HardwareMap hwMap               = null;

//    private MaxbotixUltrasonicI2c Rproximity = null;
//    private MaxbotixUltrasonicI2c Lproximity = null;
//    private MaxbotixUltrasonicI2c Fproximity = null;
//    private MaxbotixUltrasonicI2c Bproximity = null;

    public HardwareSoftware() {
    }

    public void init(final HardwareMap ahwMap) {
        hwMap = ahwMap;

        /* INITIALIZE MOTORS --------------------------------------------------------------------*/

        /* Wheel Motors */

        //Front left motor
        FLdrive  = hwMap.get(DcMotor.class, "FLdrive");

        //Front right motor
        FRdrive = hwMap.get(DcMotor.class, "FRdrive");

        //Back left motor
        BLdrive    = hwMap.get(DcMotor.class, "BLdrive");

        //Back right motor
        BRdrive   = hwMap.get(DcMotor.class, "BRdrive");

        Arm      = hwMap.get(DcMotor.class, "Arm");

        LencoderDrive = hwMap.get(DcMotor.class, "FLdrive");

        RencoderDrive = hwMap.get(DcMotor.class, "FRdrive");

        CencoderDrive = hwMap.get(DcMotor.class, "BLdrive");

        FLdrive.setDirection(DcMotor.Direction.REVERSE);  // Set to REVERSE if using AndyMark motors
        FRdrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        BLdrive.setDirection(DcMotor.Direction.REVERSE);
        BRdrive.setDirection(DcMotor.Direction.FORWARD);

        LencoderDrive.setDirection(DcMotorSimple.Direction.REVERSE); // was reverse
        RencoderDrive.setDirection(DcMotorSimple.Direction.FORWARD); // reverse was correct but flipped FRdrive
        CencoderDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hwMap.get(DcMotor.class, "Intake");
        Delivery = hwMap.get(DcMotor.class, "Delivery");

        Intake.setDirection(DcMotor.Direction.FORWARD);
        Delivery.setDirection(DcMotor.Direction.FORWARD);

        /* SET INITIAL POWER --------------------------------------------------------------------*/
        FLdrive.setPower(0);
        FRdrive.setPower(0);
        BLdrive.setPower(0);
        BRdrive.setPower(0);
        Intake.setPower(0);
        Delivery.setPower(0);
        Arm.setPower(0);

        /* SET MOTOR MODE -----------------------------------------------------------------------*/

//        LencoderDrive.setDirection(DcMotorSimple.Direction.FORWARD);
//        RencoderDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        centerEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLdrive.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FRdrive.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BLdrive.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BRdrive.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

 //       FLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 //       FRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 //       BLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 //       BRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Delivery.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //* INITIALIZE SERVOS --------------------------------------------------------------------*//*

 //      Lclaw = hwMap.get(Servo.class, "Lclaw");

       deliveryServo = hwMap.get(Servo.class, "deliveryServo");
//       armServo = hwMap.get(CRServo.class, "armServo");
       intakeServo   = hwMap.get(Servo.class, "intakeServo");
       armStableServo = hwMap.get(Servo.class, "armStableServo");

//       magnetLimitSwitch = hwMap.get(DigitalChannel.class, "sensor_digital");
//       distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
//       magnetLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

//       vs = hwMap.get(VoltageSensor.class, "vs");

//        Fproximity = hwMap.get(MaxbotixUltrasonicI2c.class, "Fproximity");
//        Bproximity = hwMap.get(MaxbotixUltrasonicI2c.class, "Bproximity");
//        Lproximity = hwMap.get(MaxbotixUltrasonicI2c.class, "Lproximity");
//        Rproximity = hwMap.get(MaxbotixUltrasonicI2c.class, "Rproximity");

    }
    /* Variables for Odometry  ------------------------------------------------------------------*/
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() { return theta; }
    public double getThetadeg() { return thetadeg; }
    public void setX(double _x) { x = _x; }
    public void setY(double _y) { y = _y; }
    public void setTheta(double _theta) { theta = _theta; }

    /* Motor getters ----------------------------------------------------------------------------*/
    public DcMotor FLdrive() { return FLdrive; }

    public DcMotor FRdrive() {
        return FRdrive;
    }

    public DcMotor BLdrive() {
        return BLdrive;
    }

    public DcMotor BRdrive() {
        return BRdrive;
    }

    public DcMotor RencoderDrive() { return RencoderDrive; }

    public DcMotor LencoderDrive() { return LencoderDrive; }

    public DcMotor CencoderDrive() { return CencoderDrive; }

    public DcMotor Intake() {
       return Intake;
   }

    public DcMotor Delivery() {
        return Delivery;
    }

    public DcMotor Arm() { return Arm; }

 //   /* Servo getters ----------------------------------------------------------------------------*/
    public Servo DeliveryServo() {
        return deliveryServo;
    }

//    public CRServo ArmServo()   { return armServo; }

    public Servo intakeServo() {
        return intakeServo;
    } // was used to flick the ring up in the intake

    public Servo ArmStableServo() { return armStableServo; }

//    public DigitalChannel MagnetLimitSwitch() { return magnetLimitSwitch; }

//    public DistanceSensor distanceSensor() { return distanceSensor; }

//    public MaxbotixUltrasonicI2c Fproximity() {return Fproximity;}
//    public MaxbotixUltrasonicI2c Bproximity() {return
//    Bproximity;}
//    public MaxbotixUltrasonicI2c Rproximity() {return Rproximity;}
//    public MaxbotixUltrasonicI2c Lproximity() {return Lproximity;}


/* More Things for Odometry that im not sure we need yet ----------------------------------------------------------------------------*/
    public void resetTicks() {
      resetLeftTicks();
      resetCenterTicks();
      resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = LencoderDrive().getCurrentPosition();
    }
    public int getLeftTicks() {
        return LencoderDrive().getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
        rightEncoderPos = RencoderDrive.getCurrentPosition();
    }
    public int getRightTicks() {
        return RencoderDrive.getCurrentPosition() - rightEncoderPos;
    }

      public void resetCenterTicks() {
        centerEncoderPos = CencoderDrive.getCurrentPosition();
    }

    public int getCenterTicks() {
        return CencoderDrive.getCurrentPosition() - centerEncoderPos;
    }

    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;      // OKAY SO WHAT TO DO IS USE CALCULATIONS FROM
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;    // SIMPLE ODO AUTO AND ROBOT CONTSTANTS TO FIND
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;  // THE NEW DELTA L AND R DISTANCES
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (((deltaLeftDistance - deltaRightDistance) / wheelDistanceApart)); // added a * .15 here to correct the theta
        thetadeg  += (((theta / 2) / Math.PI) * 360);
        LticksTelem += getLeftTicks();
        RticksTelem += getRightTicks();
        CticksTelem += getCenterTicks();
        DLeftTelem  += deltaLeftDistance;
        DRightTelem += deltaRightDistance;
        DCenterTelem += deltaCenterDistance;
        resetTicks();
    }

    public void updatePositionInches() {
//        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
//        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
//        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        previousPositionL = currentPositionL;
        previousPositionR = currentPositionR;
        previousTheta = theta;
        deltaLeftDistance = (LencoderDrive().getCurrentPosition() * RobotConstants.INCHES_PER_COUNT);     // SO WHAT TO DO NEXT is to test it again
        deltaRightDistance = (RencoderDrive().getCurrentPosition() * RobotConstants.INCHES_PER_COUNT);    // to see if it works after changing the wheel
        deltaCenterDistance = (CencoderDrive().getCurrentPosition() * RobotConstants.INCHES_PER_COUNT);   // distance apart to inches rather than meters
        currentPositionL = deltaLeftDistance;
        currentPositionR = deltaRightDistance;
//        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//        x  = (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//        y  = (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//        theta += (((deltaLeftDistance - deltaRightDistance) / wheelDistanceApartInches)); // added a * .15 here to correct the theta
        theta = (((deltaLeftDistance - deltaRightDistance) / wheelDistanceApartInches));
//        thetadeg  += (((theta / 2) / Math.PI) * 360);
        thetadeg  = (((theta / 2) / Math.PI) * 360);

        if (!((previousPositionR == currentPositionR) && (previousPositionL == currentPositionL))) {
//            if (((thetadeg >= 0) && (thetadeg <= 90)) || (thetadeg <= -270) && (thetadeg >= -360))((thetadeg <= 0) && (thetadeg >= -90))) {
//                y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//                x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//            } else if (((thetadeg >= 90) && (thetadeg <= 180)) || ((thetadeg <= -90) && (thetadeg >= -180))) {
//                y  -= (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//                x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//            } else if (((thetadeg >= 180) && (thetadeg <= 270)) || ((thetadeg <= -180) && (thetadeg >= -270))) {
//                y  -= (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//                x  -= (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//            } else if (((thetadeg >= 270) && (thetadeg <= 360)) || ((thetadeg <= -270) && (thetadeg >= -360))) {
//                y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
//                x  -= (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
//            }

                x += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta); // the x and y for these were switched before
                y += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
    }

        // LOG STUFF
        // we are having issues with the x and y values increasing when we turn back and forth, and we are going to put one axel
        // on the odo wheels to make sure that there is less creeping. 

            // ********************************************************************** //

//            currentPositionL = (LencoderDrive.getCurrentPosition());
//            currentPositionR = (RencoderDrive.getCurrentPosition());
//
//            deltaLeftDistance = currentPositionL - previousPositionL;
//            deltaRightDistance = currentPositionR - previousPositionR;
//
//            deltaTheta = ((deltaLeftDistance - deltaRightDistance) / (wheelDistanceApart));
//            theta = (theta + deltaTheta);
//
//            x = (x + (((deltaLeftDistance + deltaRightDistance) / 2) * Math.sin(theta)));
//            y = (y + (((deltaLeftDistance + deltaRightDistance) / 2) * Math.cos(theta)));
//
//            previousPositionL = currentPositionL;
//            previousPositionR = currentPositionR;
//
//        LticksTelem += getLeftTicks();
//        RticksTelem += getRightTicks();
//        CticksTelem += getCenterTicks();
//        DLeftTelem  += deltaLeftDistance;
//        DRightTelem += deltaRightDistance;
//        DCenterTelem += deltaCenterDistance;
        resetTicks();
    }

    public void updatePositionNoReset() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius / 4; // added the / 4 here because these distances were off by a factor of 4
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius / 4;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        //resetTicks();


    }

    public void stopAll() {

        FRdrive().setPower(0);
        FLdrive().setPower(0);
        BRdrive().setPower(0);
        BRdrive().setPower(0);
        Intake().setPower(0);
        Delivery().setPower(0);
        RencoderDrive().setPower(0);
        LencoderDrive().setPower(0);

    }

    // I changed the arm to continuous - Gavin
//    public void moveArm() {
//        timer.reset();
//        ArmServo().setPosition(.35);
//        sleep(1300);
//        while (timer.seconds() < 2.4) {
//            Arm().setPower(.74);
//        }
//        ArmServo().setPosition(.3);
//    }
//    public void elevateArm() {
//
//        while (MagnetLimitSwitch().getState() == true){
//            Arm().setPower(-.64);
//        }
//
//        sleep(2200);
//        Arm().setPower(0);
//        ArmDown = false;
//
//    }
//
//    public void delevateArm() {
//
//        while (MagnetLimitSwitch().getState() == true){
//            Arm().setPower(.64);
//        }
//        Arm().setPower(0);
//        ArmDown = true;
//
//    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public double getDeltaLeft(){
        return deltaLeftDistance;
    }
    public double getDeltaRight(){
       return deltaRightDistance;
    }
    public double getDeltaCenter(){
        return deltaCenterDistance;
    }

    public double getLTicksTelem() {
        return LticksTelem;
    }

    public double getRTicksTelem() {
        return RticksTelem;
    }

    public double getCTicksTelem() {
        return CticksTelem;
    }
    public double getDLeftTelem(){
        return DLeftTelem;
    }
    public double getDRightTelem(){
        return DRightTelem;
    }
    public double getDCenterTelem(){
        return DCenterTelem;
    }

}
