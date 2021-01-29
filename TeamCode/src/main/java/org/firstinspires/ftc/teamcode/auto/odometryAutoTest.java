/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pd.HardwareSoftware;
import org.firstinspires.ftc.teamcode.pd.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.tfodWebcamCommands;
import org.firstinspires.ftc.teamcode.teleOp.tfodWebcamTest;


@Autonomous(name="Simple Odo Auto", group="Pushbot")
//@Disabled
public class odometryAutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSoftware robot = new HardwareSoftware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private final tfodWebcamCommands webcam = new tfodWebcamCommands();
    private double DRIVE_SPEED = .5;
    private Double[] position = {0.0, 0.0, 0.0};
    static final double wheelDistanceApartInches = 13.583;
    static final double robotRadiusTicks = (wheelDistanceApartInches / 2) * RobotConstants.COUNTS_PER_INCH;
    private double a;
    private double b;
    private double currentAngle = 0; // 18 * Math.PI / 180
    private int case1;
    private int case2;
    private int case3;
    private int case4;
    private String ringValue;
    private static final double     P_TURN_COEFF            = 0.1;      // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.1;      // Larger is more responsive, but also less stable
    private static final double     HEADING_THRESHOLD       = 5;        // As tight as we can make it with an integer gyro

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        webcam.init(hardwareMap);

//        robot.LencoderDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RencoderDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        //encoderDrive(-.2, -20,-20,5);

//        for (int i = 0; i<5; i++) {
//            ringValue = webcam.getRingValue();
//            sleep (150);
//        }

//        ringValue = webcam.getRingValue();

//        while (opModeIsActive() && !isStopRequested()) {
//            robot.updatePositionNoReset();
//            encoderDrive(DRIVE_SPEED, 3,3,5);
//            robot.updatePositionNoReset();
//            telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//            telemetry.update();
//            //sleep(1000);
//            //fancyDrive(DRIVE_SPEED, 0, 0, 4);
//        }

        if (opModeIsActive()) {
//            encoderDrive(.5, 10, 10, 5);
//            telemetry.addData("angle", robot.getThetadeg());
//            telemetry.update();
//            sleep(500);
            //encoderTurn(.3, 10, 15, -15, 4);
//            robot.DeliveryServo().setPosition(0);


            // TESTING STUFF --------------------------------------------
//            driveByCounts(.5, -16748, 16863, 5);
//            telemetry.addData("left counts", robot.FLdrive().getCurrentPosition());
//            telemetry.addData("right counts", robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//            stop();
//            sleep(9000);
            // TESTING ST  UFF --------------------------------------------
            robot.ArmStableServo().setPosition(1);
            sleep(200);
            robot.DeliveryServo().setPosition(1);
            sleep(200);
            encoderDrive(.6, -10, -10, 5);
            sleep(200);
            encoderTurnAngle(.38, 15, 4);
            robot.intakeServo().setPosition(0);
            sleep(600);
//            ringValue = webcam.getRings();
            for (int i = 0; i < 5; i++) {
                ringValue = webcam.getRings();
                sleep(150);
            }
            telemetry.addData("ring value", ringValue);
            telemetry.update();
            sleep(1000);
            switch (ringValue) {
                case "Single": // this is the B area (the middle one)
                    telemetry.addData("theta", currentAngle);
                    telemetry.update();
                    sleep(400);
                    encoderTurnAngle(.31, 45, 4);
                    sleep(200);
                    encoderDrive(.51, -9, -9, 4);
                    sleep(200);
                    encoderTurnAngle(.49, 7.5 , 3);
                    sleep(200);
                    encoderDrive(.50, -48, -48, 4);
                    sleep(300);

                    lowerArm();

//                    encoderTurnAngle(.35, 0, 3);
//                    sleep(200);
                    encoderDrive(.5, -2, -2, 4);
                    sleep(200);
                    encoderDrive(.5, 15.7, 15.7, 4);
                    sleep(200);

                    liftArm();

                    sleep(200);
                    encoderTurnAngle(.35, 1.2, 3);  //2 is the original angle for power shot
                    sleep(200);

                    deliverRings();

//                    encoderDrive(.65, 21, 21, 4);
//                    sleep(200);
//                    encoderTurnAngle(.5, 2, 3);
//                    sleep(400);
//                    encoderDrive(.65, 13, 13, 4);
//                    sleep(200);
//                    robot.Delivery().setPower(.9);
//                    sleep(1500);
//                    robot.Intake().setPower(.8);
//                    sleep(750);

//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(1500);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(.6);

//                    sleep(3000);

                    // TODO: DEAR FUTURE GAVIN
                    //  fix     it
                    //  what do we need to uh we need to fix the shooter and
                    //   wobble arm,,, that's basically what we need to do
                    //     would you agree  ?
                    //  'yes'
                    //

//                    robot.Intake().setPower(0);
//                    robot.Delivery().setPower(0);
//                    sleep(200);
                    encoderDrive(.65, -8, -8, 4);
//                    telemetry.addData("ringValue in switch", "Single");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    sleep(4000);
                    break;
                case "Quad": // this is the C area (farthest one)

                    sleep(500);
//                    encoderDrive(.65, -70, -70, 4);
//                    encoderDrive(.2, 10, 10, 5);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
                    telemetry.addData("ringValue in switch", "None");
                    telemetry.addData("ringValue actual", ringValue);
                    telemetry.update();
                    // sleep(4000);

                    encoderTurnAngle(.31, -45, 4);
                    sleep(300);
                    encoderDrive(.65, -12, -12, 4);
                    sleep(300);
                    encoderTurnAngle(.3, 5, 3);
                    sleep(200);
                    encoderDrive(.65, -72, -72, 4);
                    sleep(400);
                    lowerArm();

                    encoderDrive(.65, 2, 2, 4);

                    liftArm();
                    sleep(200);
//                    encoderDrive(.65, 36,36, 4);
//                    sleep(200);
                    encoderTurnAngle(.5, -20, 3);//angle -50
                    sleep(200);
                    encoderDrive(.65, 46, 46, 4);
                    sleep(200);
                    encoderTurnAngle(.35, 2.5, 3); //angle 16 - we are now using the angle to hit the high goal
                    encoderDrive(.5, -1.7, -1.7, 4);
                    sleep(200);
//                    sleep(200);
//                    encoderDrive(.65, -15, -15, 4);
//                    sleep(200);

                    deliverRings();
                    encoderDrive(.65, -11, -11, 4);
//                    encoderTurnAngle4(.2, -45, 6);
//                    sleep(300);
//                    encoderDrive(.40, -30, -30, 4);
//                    sleep(300);
//                    encoderTurnAngle4(.3, -8, 3);
//                    sleep(200);
//                    encoderDrive(.50, -75, -75, 4);
//                    sleep(400);
//                    encoderDrive(.50, 39,39, 4);
//                    sleep(200);
//                    encoderTurnAngle4(.5, -50, 3);
//                    sleep(200);
//                    encoderDrive(.65, 24, 24, 4);
//                    sleep(400);
//                    encoderTurnAngle4(.35, 16, 3);
//                    sleep(200);
//                    encoderDrive(.65, 8, 8, 4);
//                    sleep(200);
//
//                    robot.Delivery().setPower(.83);
//                    sleep(1500);
//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(800);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(800);
//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(800);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(800);
//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(800);
//                    robot.Delivery().setPower(0);
//                    sleep(200);
//                    encoderDrive(.65, -21, -21, 4);
////                    encoderDrive(.5, 41.5, 41.5, 4);
////                    sleep(200);
////                    encoderTurnAngle(.35, 2, 3);
////                    sleep(200);
////                    encoderDrive(.6, 10, 10, 5);
////                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//                    telemetry.addData("ringValue in switch", "Quad");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    sleep(4000);
                    break;
                case "None": // this is the A area (closest one)
                    //encoderTurnAngle(.3, 36, 4);
                    sleep(500);
//                    encoderDrive(.65, -70, -70, 4);
//                    encoderDrive(.2, 10, 10, 5);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
                    telemetry.addData("ringValue in switch", "None");
                    telemetry.addData("ringValue actual", ringValue);
                    telemetry.update();
                   // sleep(4000);

                    encoderTurnAngle(.31, -45, 4);
                    sleep(300);
                    encoderDrive(.65, -12, -12, 4);
                    sleep(300);
                    encoderTurnAngle(.3, 7, 3);
                    sleep(200);
                    encoderDrive(.65, -19, -19, 4);
                    sleep(400);
                    lowerArm();
//                    encoderDrive(.65, 36,36, 4);
//                    sleep(200);
                    encoderDrive(.65, 2.5, 2.5, 4);
                    sleep(200);
                    liftArm();
                    sleep(200);
                    encoderTurnAngle(.5, -88, 3);//angle -50
                    sleep(200);
                    encoderDrive(.65, 16.7, 16.7, 4);
                    sleep(200);
                    encoderTurnAngle(.35, 2, 3); //angle 16 - we are now using the angle to hit the high goal
                    sleep(300);
                    encoderDrive(.5, -12.5, -12.5, 4);
                    sleep(200);
//                    encoderTurnAngle(.35, 4, 3);
                    sleep(200);
//                    sleep(200);
//                    encoderDrive(.65, -15, -15, 4);
//                    sleep(200);

                    deliverRings();

//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(.6);
//                    sleep(1500);
//                    robot.DeliveryServo().setPosition(0);
//                    sleep(1000);
//                    robot.DeliveryServo().setPosition(.6);
                    encoderDrive(.65, -11, -11, 4);


                    break;
                default:
                    encoderTurnAngle(.3, 0, 4);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
                    telemetry.addData("ringValue in switch", "Default");
                    telemetry.addData("ringValue actual", ringValue);
                    telemetry.update();
                    sleep(4000);
                    //sleep(500);
                    //encoderTurn(.3, -10, 10,10, 4);
//            }
//            sleep(2000);
//            encoderTurnAngle(.4, 30, 4);
//            telemetry.addData("angle", robot.getThetadeg());
//            telemetry.update();
//            sleep(2000);
//            encoderDrive(.4, 10, 10, 5);
//            telemetry.update();
//            telemetry.addData("angle", robot.getThetadeg());
//            sleep(2000);
//            encoderTurnAngle(.4, 0, 4);
//            encoderTurnAngle(.5, 90, 4);
//            telemetry.addData("angle", robot.getThetadeg());
//            telemetry.update();
//            sleep(100);
//            encoderDrive(.2, 10, 10, 5);
//            telemetry.addData("angle", robot.getThetadeg());
//            telemetry.update();
//            sleep(500);
//            encoderTurnAngle(.5, -30, 4);
//            telemetry.addData("angle", robot.getThetadeg());
//            telemetry.update();
            }

//        if (opModeIsActive()) {
//
//            switch(ringValue) {
//                case "Single":
//                    encoderDrive(-.2, -10,-10,5);
//                    sleep(500);
//                    encoderTurn(.3, -10, 15,-15, 4);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//                    telemetry.addData("ringValue in switch", "Single");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    break;
//                case "Quad":
//                    encoderDrive(-.2, -15,-15,5);
//                    sleep(500);
//                    encoderTurn(.3, -10, 10,10, 4);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//                    telemetry.addData("ringValue in switch", "Quad");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    break;
//                case "None":
//                    encoderDrive(-.2, -20,-20,5);
//                    sleep(500);
//                    encoderTurn(.3, -10, 10,10, 4);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//                    telemetry.addData("ringValue in switch", "None");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    break;
//                default:
//                    encoderDrive(-.2, -20,-20,5);
//                    telemetry.addData("position:", robot.getX() + ", " + robot.getY());
//                    telemetry.addData("ringValue in switch", "Default");
//                    telemetry.addData("ringValue actual", ringValue);
//                    telemetry.update();
//                    //sleep(500);
//                    //encoderTurn(.3, -10, 10,10, 4);
//            }
//
////            robot.updatePositionNoReset();
////            encoderDrive(-.2, -15,-15,5);
////            sleep(500);
////            encoderTurn(.3, -10, 10,10, 4);
////            robot.updatePositionNoReset();
////            telemetry.addData("position:", robot.getX() + ", " + robot.getY());
////            telemetry.update();
//            //sleep(1000);
//            //fancyDrive(DRIVE_SPEED, 0, 0, 4);
//        }
        }
    }

    public void driveByCounts(double speed,
                      double leftCounts, double rightCounts,
                      double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        runtime.reset();


        if(opModeIsActive() && runtime.time() < timeoutS) {
            newLeftTarget = (int) leftCounts;
            newRightTarget = (int) rightCounts;

            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.FLdrive().getCurrentPosition(),
                    robot.FRdrive().getCurrentPosition());
            telemetry.update();

//            sleep(3000);

//            robot.FLdrive().setPower(-speed);
//            robot.FRdrive().setPower(-speed);
//            robot.BRdrive().setPower(-speed);
//            robot.BLdrive().setPower(-speed);

            if (leftCounts < 0 && rightCounts > 0) {
                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(-speed);
                    robot.FRdrive().setPower(-speed);
                    robot.BRdrive().setPower(-speed);
                    robot.BLdrive().setPower(-speed);
                    speed -= .01;
                }
            } else if (leftCounts > 0 && rightCounts < 0) {
                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(speed);
                    robot.FRdrive().setPower(speed);
                    robot.BRdrive().setPower(speed);
                    robot.BLdrive().setPower(speed);
                    speed -= .01;
                }
            }

            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);
        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        runtime.reset();

//         Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.LencoderDrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
//            newRightTarget = robot.RencoderDrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.LencoderDrive().getCurrentPosition(),
//                    robot.RencoderDrive().getCurrentPosition());
//            telemetry.update();
//            sleep(3000);
//            robot.LencoderDrive().setTargetPosition(newLeftTarget);
//            robot.RencoderDrive().setTargetPosition(newRightTarget);
//
////             the original initial program of the teleop category that we will implement on this year's ultimate goal game
//
//            // Turn On RUN_TO_POSITION
////            robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.LencoderDrive().setPower(Math.abs(speed));
//            robot.RencoderDrive().setPower(Math.abs(speed));
//            robot.BRdrive().setPower(Math.abs(speed));
//            robot.BLdrive().setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.LencoderDrive().isBusy() && robot.RencoderDrive().isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
//                        robot.LencoderDrive().getCurrentPosition(),
//                        robot.RencoderDrive().getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.LencoderDrive().setPower(0);
//            robot.RencoderDrive().setPower(0);
//            robot.BLdrive().setPower(0);
//            robot.BRdrive().setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.LencoderDrive().getCurrentPosition(),
//                    robot.RencoderDrive().getCurrentPosition());
//            telemetry.update();
//            sleep(3000);


        //----------------------------------------------------------------------------------------//
        //                            |                          |                                //
        //                         //////                      //////                             //
        //                                          ^                                             //
        //                                    --------------                                      //
        //----------------------------------------------------------------------------------------//


//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
//            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.FLdrive().getCurrentPosition(),
//                    robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//            sleep(3000);
//
//            // SOMETHING TO TEST: make it run to position without using run to position
//
//            robot.FLdrive().setTargetPosition(newLeftTarget);
//            robot.FRdrive().setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            //robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            //robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.FLdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FRdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.FLdrive().setPower(speed);
//            robot.FRdrive().setPower(speed);
//            robot.BRdrive().setPower(speed);
//            robot.BLdrive().setPower(speed);
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.LencoderDrive().isBusy() && robot.RencoderDrive().isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
//                        robot.FLdrive().getCurrentPosition(),
//                        robot.FRdrive().getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.FLdrive().setPower(0);
//            robot.FRdrive().setPower(0);
//            robot.BLdrive().setPower(0);
//            robot.BRdrive().setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.FLdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.FRdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.FLdrive().getCurrentPosition(),
//                    robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//            sleep(3000);
//
//
//
//
//              sleep(250);   // optional pause after each move
//        }



        //----------------------------------------------------------------------------------------//
        //                            |                          |                                //
        //                         //////                      //////                             //
        //                                          ^                                             //
        //                                    --------------                                      //
        //----------------------------------------------------------------------------------------//



        if(opModeIsActive() && runtime.time() < timeoutS) {
            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);

            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.FLdrive().getCurrentPosition(),
                    robot.FRdrive().getCurrentPosition());
            telemetry.update();

//            sleep(3000);

//            robot.FLdrive().setPower(-speed);
//            robot.FRdrive().setPower(-speed);
//            robot.BRdrive().setPower(-speed);
//            robot.BLdrive().setPower(-speed);

            if (leftInches > 0 && rightInches > 0) {
                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(-speed);
                    robot.FRdrive().setPower(-speed);
                    robot.BRdrive().setPower(-speed);
                    robot.BLdrive().setPower(-speed);
                }
            } else if (leftInches < 0 && rightInches < 0) {
                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(speed);
                    robot.FRdrive().setPower(speed);
                    robot.BRdrive().setPower(speed);
                    robot.BLdrive().setPower(speed);
                }
            }

            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);
        }
    }
    public void encoderTurn(double speed, double targetAngle,
                            double leftInches, double rightInches,
                            double timeoutS){

        int newLeftTarget;
        int newRightTarget;
        double currentAngle = robot.getTheta();


        if (opModeIsActive() && runtime.time() < timeoutS){
            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);

            if(currentAngle < targetAngle){
                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(-speed); // negatives were reversed because the encoders were reversed
                    robot.FRdrive().setPower(speed);
                    robot.BRdrive().setPower(speed);
                    robot.BLdrive().setPower(-speed);
                }

            }else if (currentAngle > targetAngle){
                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();

                    robot.FLdrive().setPower(speed); // negatives here were also reversed
                    robot.FRdrive().setPower(-speed);
                    robot.BRdrive().setPower(-speed);
                    robot.BLdrive().setPower(speed);
                }
            }

            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);


        }

//        if(opModeIsActive()) {
//            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
//            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
//
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.FLdrive().getCurrentPosition(),
//                    robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//
//            sleep(3000);
//
//            robot.FLdrive().setPower(speed);
//            robot.FRdrive().setPower(speed);
//            robot.BRdrive().setPower(speed);
//            robot.BLdrive().setPower(speed);
//
//            if (leftInches > 0 && rightInches > 0) {
//                while ((robot.FLdrive().getCurrentPosition() < newLeftTarget) || (robot.FRdrive().getCurrentPosition() < newRightTarget)) {
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.FLdrive().getCurrentPosition(),
//                            robot.FRdrive().getCurrentPosition());
//                    telemetry.update();
//                }
//            }else if (leftInches < 0 && rightInches < 0){
//                while ((robot.FLdrive().getCurrentPosition() > newLeftTarget) || (robot.FRdrive().getCurrentPosition() > newRightTarget)) {
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.FLdrive().getCurrentPosition(),
//                            robot.FRdrive().getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//
//            robot.FLdrive().setPower(0);
//            robot.FRdrive().setPower(0);
//            robot.BLdrive().setPower(0);
//            robot.BRdrive().setPower(0);
//
//        }

    }

    public void encoderTurnAngle( double speed, double angle, double timeoutS) {
        double distance;
        double deltaAngle;
        int newLeftTarget;
        int newRightTarget;
        //double currentAngle = robot.getTheta();
        runtime.reset();

        if (opModeIsActive() && runtime.time() < timeoutS) {
//            angle = angle * Math.PI / 180;
//            deltaAngle = angle - currentAngle;
//            distance = deltaAngle * robotRadiusTicks;

            deltaAngle = angle - currentAngle;
            deltaAngle = deltaAngle * Math.PI / 180;
            distance = deltaAngle * robotRadiusTicks;

            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int)distance;
            newRightTarget = robot.FRdrive().getCurrentPosition() - (int)distance;

            if(distance > 0){
                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(-speed); // negatives were reversed because the encoders were reversed
                    robot.FRdrive().setPower(speed);
                    robot.BRdrive().setPower(speed);
                    robot.BLdrive().setPower(-speed);
                }

            }else if (distance < 0){
                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();

                    robot.FLdrive().setPower(speed); // negatives here were also reversed
                    robot.FRdrive().setPower(-speed);
                    robot.BRdrive().setPower(-speed);
                    robot.BLdrive().setPower(speed);
                }
            }
            robot.FLdrive().setPower(0); // negatives were reversed because the encoders were reversed
            robot.FRdrive().setPower(0);
            robot.BRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            currentAngle = angle;
        }
    }

    public void encoderTurnAngle4( double speed, double angle, double timeoutS) {
        double distance;
        double deltaAngle;
        int newLeftTarget;
        int newRightTarget;
        //double currentAngle = robot.getTheta();
        runtime.reset();

        if (opModeIsActive() && runtime.time() < timeoutS) {
            deltaAngle = angle - currentAngle;
            deltaAngle = deltaAngle * Math.PI / 180;
            distance = deltaAngle * robotRadiusTicks;

            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int)distance;
            newRightTarget = robot.FRdrive().getCurrentPosition() - (int)distance;

            if(distance > 0){
                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();
                    robot.FLdrive().setPower(-speed); // negatives were reversed because the encoders were reversed
                    robot.FRdrive().setPower(speed);
                    robot.BRdrive().setPower(speed);
                    robot.BLdrive().setPower(-speed);
                    currentAngle = angle;
                }

            }else if (distance < 0){
                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
                        && opModeIsActive() && runtime.time() < timeoutS) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.FLdrive().getCurrentPosition(),
                            robot.FRdrive().getCurrentPosition());
                    telemetry.update();

                    robot.FLdrive().setPower(speed); // negatives here were also reversed
                    robot.FRdrive().setPower(-speed);
                    robot.BRdrive().setPower(-speed);
                    robot.BLdrive().setPower(speed);
                    currentAngle = angle;
                }
            }
            robot.FLdrive().setPower(0); // negatives were reversed because the encoders were reversed
            robot.FRdrive().setPower(0);
            robot.BRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            currentAngle = angle;
        }
    }

    public void fancyDrive ( double speed,
                             double xTarget, double yTarget,
                             double timeoutS){
        int newLeftTarget;
        int newRightTarget;
        double distance;
        double x;
        double y;
        double adjacent;
        double newXTarget;
        double newYTarget;
        double hypotenuse;
        double theta;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            x = robot.getX();
            y = robot.getY();
            distance = Math.sqrt(((xTarget-x)*(xTarget-x)) + ((yTarget-y)*(yTarget-y)));
            telemetry.addData(" ", distance + " in");

//                adjacent = (xTarget-x); //can be (yTarget-y) too
//                hypotenuse = distance;
//                theta = Math.cosh(adjacent/hypotenuse);

            theta = Math.atan2(yTarget-y, xTarget-x);

            gyroTurn(.4, theta, 6);
            sleep(100);

            newXTarget = distance;
            newYTarget = distance;

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.LencoderDrive().getCurrentPosition() + (int) (newXTarget * RobotConstants.COUNTS_PER_INCH);
            newRightTarget = robot.RencoderDrive().getCurrentPosition() + (int) (newYTarget * RobotConstants.COUNTS_PER_INCH);
            robot.LencoderDrive().setTargetPosition(newLeftTarget);
            robot.RencoderDrive().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LencoderDrive().setPower(Math.abs(speed));
            robot.RencoderDrive().setPower(Math.abs(speed));
            robot.BRdrive().setPower(Math.abs(speed));
            robot.BLdrive().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.LencoderDrive().isBusy() && robot.RencoderDrive().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.LencoderDrive().getCurrentPosition(),
                        robot.RencoderDrive().getCurrentPosition());
                telemetry.update();
            }
            while (opModeIsActive()) {
                robot.updatePositionNoReset();
                position[0] = robot.getX();
                position[1] = robot.getY();
                Math.abs(position[0]);
            }

            // Stop all motion;
            robot.LencoderDrive().setPower(0);
            robot.RencoderDrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
    public void gyroTurn (  double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = //getError(angle);
//                angle - robot.getGyro().getHeading();
                angle - robot.getTheta();
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            // steer will be positive or negative dependingon the direction we need to go
            steer = getSteer(error, PCoeff);

            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;

//            leftSpeed = speed * steer;
//            rightSpeed = -leftSpeed;

        }

        // Send desired speeds to motors.
        robot.LencoderDrive().setPower(leftSpeed);
        robot.RencoderDrive().setPower(rightSpeed);
        robot.BLdrive().setPower(leftSpeed);
        robot.BRdrive().setPower(rightSpeed);
        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Heading", "%d", robot.getGyro().getHeading());
        telemetry.addData("Heading", "%d", robot.getTheta());
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void fancyDriveSteer ( double speed,
                                  double xTarget, double yTarget,
                                  double timeoutS){
        int newLeftTarget;
        int newRightTarget;
        double distance;
        double x;
        double y;
        double adjacent;
        double newXTarget;
        double newYTarget;
        double hypotenuse;
        double theta;

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            x = robot.getX();
            y = robot.getY();
            distance = Math.sqrt(((xTarget-x)*(xTarget-x)) + ((yTarget-y)*(yTarget-y)));
            telemetry.addData(" ", distance + " in");

//                adjacent = (xTarget-x); //can be (yTarget-y) too
//                hypotenuse = distance;
//                theta = Math.cosh(adjacent/hypotenuse);

            theta = Math.atan2(yTarget-y, xTarget-x);

            gyroTurn(.4, theta, 6);
            sleep(100);

            newXTarget = distance;
            newYTarget = distance;

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.LencoderDrive().getCurrentPosition() + (int) (newXTarget * RobotConstants.COUNTS_PER_INCH);
            newRightTarget = robot.RencoderDrive().getCurrentPosition() + (int) (newYTarget * RobotConstants.COUNTS_PER_INCH);
            robot.LencoderDrive().setTargetPosition(newLeftTarget);
            robot.RencoderDrive().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LencoderDrive().setPower(Math.abs(speed));
            robot.RencoderDrive().setPower(Math.abs(speed));
            robot.BRdrive().setPower(Math.abs(speed));
            robot.BLdrive().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.LencoderDrive().isBusy() && robot.RencoderDrive().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.LencoderDrive().getCurrentPosition(),
                        robot.RencoderDrive().getCurrentPosition());
                telemetry.update();

                error = getError(theta); //we had to do this because it was over-rotating by 5.56% of the intended angle
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.LencoderDrive().setPower(leftSpeed);
                robot.RencoderDrive().setPower(rightSpeed);
                robot.BLdrive().setPower(leftSpeed);
                robot.BRdrive().setPower(rightSpeed);
            }
            while (opModeIsActive()) {
                robot.updatePositionNoReset();
                position[0] = robot.getX();
                position[1] = robot.getY();
                Math.abs(position[0]);
            }

            // Stop all motion;
            robot.LencoderDrive().setPower(0);
            robot.RencoderDrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RencoderDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    private double getError(double inAngle) {

        // calculate error in -179 to +180 range
        // Don't use the integrated Z Value because that is not adjusted based upon the calibration
        //robotError = targetAngle - m_gyro.getIntegratedZValue();
        //int heading = robot.getGyro().getHeading();  // return a value between 0 and 360

        int heading = (int)robot.getTheta();

        // Reduce the input angle to within range o to 360 (for input angles that are negative)
        double targetAngle = (inAngle < 0 )?inAngle+360:inAngle;

        // Figure out the error angle
        double robotError = targetAngle - heading;

        // Figure out the error angle
        if (robotError < -180) {
            robotError += 360;

        } else if (robotError > 180) { // target angle is greater than 180
            robotError -= 360;
        }
        // This returns a value between -179 to -1 (to turn counter clockwise)
        // and 1 to 180 (to turn clockwise)
        return robotError;
    }

    private void lowerArm() {
        robot.ArmStableServo().setPosition(1);
        sleep(350);
        robot.Arm().setPower(.62);
//        robot.ArmServo().setPower(-.8);
        sleep(2500);
        robot.Arm().setPower(0);
        sleep(350);
//        robot.ArmServo().setPower(-.8);
//        sleep(1650);
        robot.ArmStableServo().setPosition(0);
        sleep(500);
    }
    private void liftArm() {
        robot.ArmStableServo().setPosition(1);
        sleep(350);
        robot.Arm().setPower(-.62);
        sleep(2500);
        robot.Arm().setPower(0);
    }

    private void deliverRings() {
        robot.Delivery().setPower(.9);
        sleep(1500);
        robot.Intake().setPower(.9);
        sleep(2200);
        robot.Intake().setPower(0);
        robot.Delivery().setPower(0);
        sleep(200);
    }

}
