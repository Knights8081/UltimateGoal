package org.firstinspires.ftc.teamcode.pd;
import org.firstinspires.ftc.teamcode.pd.HardwareSoftware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;



// this is a class used for driving (duh its in the name) and for strafing and moving diagonally and turing
// basically copied Luke's code from 2018 - 2019 season


public class Driving {

    // this is luke's code just renamed variables
    public static void drive(final HardwareSoftware robot, final double drive, final double strafe, final double rotate) {
        double FLpower = drive + strafe + rotate;
        double FRpower = drive - strafe - rotate;
        double BLpower = drive - strafe + rotate;
        double BRpower = drive + strafe - rotate;

        robot.FLdrive().setPower(.9 * FLpower);
        robot.FRdrive().setPower(.9 * FRpower);
        robot.BLdrive().setPower(.9 * BLpower);
        robot.BRdrive().setPower(.9 * BRpower);
    }
    // method for the old code or the original code we initially used
    // I have no idea if this will work lol
    public static void oldDrive(final HardwareSoftware robot) {
        if (gamepad1.left_stick_y > .1) {
            robot.FLdrive().setPower(-gamepad1.left_stick_y);
            robot.FRdrive().setPower(-gamepad1.left_stick_y);
            robot.BLdrive().setPower(-gamepad1.left_stick_y);
            robot.BRdrive().setPower(-gamepad1.left_stick_y);
            telemetry.update();
        } else if (gamepad1.left_stick_y < -.1) {
            robot.FLdrive().setPower(-gamepad1.left_stick_y);
            robot.FRdrive().setPower(-gamepad1.left_stick_y);
            robot.BLdrive().setPower(-gamepad1.left_stick_y);
            robot.BRdrive().setPower(-gamepad1.left_stick_y);
            telemetry.update();
        } else if (gamepad1.right_stick_x > .1) {
            robot.FLdrive().setPower(gamepad1.right_stick_x);
            robot.FRdrive().setPower(-gamepad1.right_stick_x);
            robot.BLdrive().setPower(gamepad1.right_stick_x);
            robot.BRdrive().setPower(-gamepad1.right_stick_x);
            telemetry.update();
        } else if (gamepad1.right_stick_x < -.1) {
            robot.FLdrive().setPower(gamepad1.right_stick_x);
            robot.FRdrive().setPower(-gamepad1.right_stick_x);
            robot.BLdrive().setPower(gamepad1.right_stick_x);
            robot.BRdrive().setPower(-gamepad1.right_stick_x);
            telemetry.update();
        }
        else if (gamepad1.left_stick_x > .1) {
            robot.FLdrive().setPower(-gamepad1.left_stick_x);
            robot.FRdrive().setPower(gamepad1.left_stick_x);
            robot.BLdrive().setPower(gamepad1.left_stick_x);
            robot.BRdrive().setPower(-gamepad1.left_stick_x);
            telemetry.update();
        } else if (gamepad1.left_stick_x < -.1) {
            robot.FLdrive().setPower(-gamepad1.left_stick_x);
            robot.FRdrive().setPower(gamepad1.left_stick_x);
            robot.BLdrive().setPower(gamepad1.left_stick_x);
            robot.BRdrive().setPower(-gamepad1.left_stick_x);
            telemetry.update();
        }else {
            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);
        }
    }



}
