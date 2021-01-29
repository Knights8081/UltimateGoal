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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pd.HardwareSoftware;
import org.firstinspires.ftc.teamcode.pd.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.tfodWebcamCommands;


@Autonomous(name="Auto test run to position", group="Pushbot")
//@Disabled
public class autonomousTestRunToPosition extends LinearOpMode {

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

        robot.FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(.2, 10, 10, 10);
            sleep(600);
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


        if (opModeIsActive()) {

            robot.FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.FLdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.FLdrive().getCurrentPosition(),
                    robot.FRdrive().getCurrentPosition());
            telemetry.update();
            sleep(3000);

            // SOMETHING TO TEST: make it run to position without using run to position

            robot.FLdrive().setTargetPosition(-newLeftTarget);
            robot.FRdrive().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            robot.FLdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FRdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.BLdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.BRdrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FLdrive().setPower(-speed);
            robot.FRdrive().setPower(-speed);
            robot.BRdrive().setPower(-speed);
            robot.BLdrive().setPower(-speed);
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
                        robot.FLdrive().getCurrentPosition(),
                        robot.FRdrive().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FLdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRdrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.FLdrive().getCurrentPosition(),
                    robot.FRdrive().getCurrentPosition());
            telemetry.update();
            sleep(3000);




              sleep(250);   // optional pause after each move
        }



        //----------------------------------------------------------------------------------------//
        //                            |                          |                                //
        //                         //////                      //////                             //
        //                                          ^                                             //
        //                                    --------------                                      //
        //----------------------------------------------------------------------------------------//



//        if(opModeIsActive() && runtime.time() < timeoutS) {
//            newLeftTarget = robot.FLdrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
//            newRightTarget = robot.FRdrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
//
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d",
//                    robot.FLdrive().getCurrentPosition(),
//                    robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//
////            sleep(3000);
//
////            robot.FLdrive().setPower(-speed);
////            robot.FRdrive().setPower(-speed);
////            robot.BRdrive().setPower(-speed);
////            robot.BLdrive().setPower(-speed);
//
//            if (leftInches > 0 && rightInches > 0) {
//                while (((robot.FLdrive().getCurrentPosition() < newLeftTarget) && (robot.FRdrive().getCurrentPosition() < newRightTarget))
//                        && opModeIsActive() && runtime.time() < timeoutS) {
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.FLdrive().getCurrentPosition(),
//                            robot.FRdrive().getCurrentPosition());
//                    telemetry.update();
//                    robot.FLdrive().setPower(-speed);
//                    robot.FRdrive().setPower(-speed);
//                    robot.BRdrive().setPower(-speed);
//                    robot.BLdrive().setPower(-speed);
//                }
//            }else if (leftInches < 0 && rightInches < 0){
//                while (((robot.FLdrive().getCurrentPosition() > newLeftTarget) && (robot.FRdrive().getCurrentPosition() > newRightTarget))
//                        && opModeIsActive() && runtime.time() < timeoutS) {
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.FLdrive().getCurrentPosition(),
//                            robot.FRdrive().getCurrentPosition());
//                    telemetry.update();
//                    robot.FLdrive().setPower(speed);
//                    robot.FRdrive().setPower(speed);
//                    robot.BRdrive().setPower(speed);
//                    robot.BLdrive().setPower(speed);
//                }
//            }
//
//            robot.FLdrive().setPower(0);
//            robot.FRdrive().setPower(0);
//            robot.BLdrive().setPower(0);
//            robot.BRdrive().setPower(0);
//
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
            angle = angle * Math.PI / 180;
            deltaAngle = angle - currentAngle;
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




}
