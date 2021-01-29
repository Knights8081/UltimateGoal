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

package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pd.Driving;
import org.firstinspires.ftc.teamcode.pd.HardwareSoftware;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="1st Teleop", group="Iterative Opmode")
//@Disabled
public class TOIPOTTCTWWIOTYUGG extends OpMode
    // 257
{

    private final HardwareSoftware robot = new HardwareSoftware();
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        robot.updatePositionNoReset();

        /*DRIVING-----------------------------------------------------------*/
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Driving.drive(robot, drive, strafe, rotate);

//        if (gamepad1.left_stick_y > .1) {
//            robot.FLdrive().setPower(-gamepad1.left_stick_y);
//            robot.FRdrive().setPower(-gamepad1.left_stick_y);
//            robot.BLdrive().setPower(-gamepad1.left_stick_y);
//            robot.BRdrive().setPower(-gamepad1.left_stick_y);
//            telemetry.update();
//        } else if (gamepad1.left_stick_y < -.1) {
//            robot.FLdrive().setPower(-gamepad1.left_stick_y);
//            robot.FRdrive().setPower(-gamepad1.left_stick_y);
//            robot.BLdrive().setPower(-gamepad1.left_stick_y);
//            robot.BRdrive().setPower(-gamepad1.left_stick_y);
//            telemetry.update();
//        } else if (gamepad1.right_stick_x > .1) {
//            robot.FLdrive().setPower(gamepad1.right_stick_x);
//            robot.FRdrive().setPower(-gamepad1.right_stick_x);
//            robot.BLdrive().setPower(gamepad1.right_stick_x);
//            robot.BRdrive().setPower(-gamepad1.right_stick_x);
//            telemetry.update();
//        } else if (gamepad1.right_stick_x < -.1) {
//            robot.FLdrive().setPower(gamepad1.right_stick_x);
//            robot.FRdrive().setPower(-gamepad1.right_stick_x);
//            robot.BLdrive().setPower(gamepad1.right_stick_x);
//            robot.BRdrive().setPower(-gamepad1.right_stick_x);
//            telemetry.update();
//        }
//        else if (gamepad1.left_stick_x > .1) {
//            robot.FLdrive().setPower(-gamepad1.left_stick_x);
//            robot.FRdrive().setPower(gamepad1.left_stick_x);
//            robot.BLdrive().setPower(gamepad1.left_stick_x);
//            robot.BRdrive().setPower(-gamepad1.left_stick_x);
//            telemetry.update();
//        } else if (gamepad1.left_stick_x < -.1) {
//            robot.FLdrive().setPower(-gamepad1.left_stick_x);
//            robot.FRdrive().setPower(gamepad1.left_stick_x);
//            robot.BLdrive().setPower(gamepad1.left_stick_x);
//            robot.BRdrive().setPower(-gamepad1.left_stick_x);
//            telemetry.update();
//        } else {
//            robot.FLdrive().setPower(0);
//            robot.FRdrive().setPower(0);
//            robot.BLdrive().setPower(0);
//            robot.BRdrive().setPower(0);
//        }


        /* INTAKE---------------------------------------------------------------*/
        if (gamepad1.left_trigger > .1){                         // outtakes
            robot.Intake().setPower(-gamepad1.left_trigger);

        } else if (gamepad1.right_trigger > .1){                 // intakes
            robot.Intake().setPower(gamepad1.right_trigger);

        } else {
            robot.Intake().setPower(0);
            telemetry.update();
        }


        /*DELIVERY------------------------------------------------------*/
        if (gamepad1.a){
            robot.Delivery().setPower(.9);
//            if (gamepad2.left_bumper){
//                robot.DeliveryServo().setPosition(.6);
//            }else if (gamepad2.right_bumper){
//                robot.DeliveryServo().setPosition(0);
//            }
        } else if (gamepad2.a){
            robot.Delivery().setPower(.9);
//            if (gamepad2.left_bumper){
//                robot.DeliveryServo().setPosition(.6);
//            }else if (gamepad2.right_bumper){
//                robot.DeliveryServo().setPosition(0);
//            }
        }else if (gamepad2.b){
            robot.Delivery().setPower(.8);

        }else if (gamepad1.b){
            robot.Delivery().setPower(.7);


        } else if (gamepad1.x) {                 // REVERSES delivery
            robot.Delivery().setPower(-.3);
        }

        else {
            robot.Delivery().setPower(0);
        }

        if (gamepad1.right_bumper) {        // lowers it
            robot.DeliveryServo().setPosition(0);
        } else if (gamepad1.left_bumper) {  // uppers it
            robot.DeliveryServo().setPosition(1);
        }
        if (gamepad2.x){      // lowers it
            robot.DeliveryServo().setPosition(1);
        }else if (gamepad2.y){   // uppers it
            robot.DeliveryServo().setPosition(0);
        }

//        if (gamepad2.left_stick_y > .1) {
//            robot.Arm().setPower(gamepad2.left_stick_y);
//        } else if (gamepad2.left_stick_y < -.1) {
//            robot.Arm().setPower(gamepad2.left_stick_y);
//        } else {
//            robot.Arm().setPower(0);
//        }


        /*Wobble Arm------------------------------------------------------*/
        if (gamepad2.left_trigger > .1) {
            robot.Arm().setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > .1) {
            robot.Arm().setPower(-gamepad2.right_trigger);
        } else {
            robot.Arm().setPower(0);
        }

        if (gamepad2.left_bumper) {            // opens it
            robot.ArmStableServo().setPosition(.3);
        } else if (gamepad2.right_bumper) {    // closes it
            robot.ArmStableServo().setPosition(1.0);
        }

        if (gamepad1.dpad_left) {              // puts the servo back
            robot.intakeServo().setPosition(1);
        } else if (gamepad1.dpad_right) {      // pushes it forward
            robot.intakeServo().setPosition(0);
        }

        /*Magnetic Limit Switch-------------------------------------------*/
//        if (robot.MagnetLimitSwitch().getState() == true) {
//            telemetry.addData("Digital Touch", "Is Not Pressed");
//        } else {
//            telemetry.addData("Digital Touch", "Is Pressed");
//        }

//        if (gamepad2.a) {            // was dpad_left
//            robot.ArmServo().setPower(.75);     // winds the arm
//        } else if (gamepad2.b) {     // was dpad_right
//            robot.ArmServo().setPower(-.75);    // unwinds the arm
//        } else {
//            robot.ArmServo().setPower(0);
//        }

//        if (gamepad2.a) {
//            robot.moveArm();
//        }

//        if (gamepad1.x) {
//            telemetry.addData("Silly number", robot.FLdrive().getCurrentPosition());
//            telemetry.addData("Right silly number", robot.FRdrive().getCurrentPosition());
//            telemetry.update();
//        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

//    public void moveArm() {
//        timer.reset();
//        robot.ArmServo().setPosition(.3);
//        while (timer.seconds() < 1.5) {
//            robot.Arm().setPower(-.2);
//        }
//    }
}
