package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pd.HardwareSoftware;
import org.firstinspires.ftc.teamcode.pd.RobotConstants;

@TeleOp(name = "odo print position", group="Iterative Opmode")
//@Disabled
public class odoPrintPosition extends OpMode
{
    //private ElapsedTime runtime = new ElapsedTime();
    private HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init () {
        telemetry.addData("status", "initialized");
        robot.init(hardwareMap);
        //robot.updatePosition();
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop () {
        //robot.updatePositionNoReset();
        //robot.updatePosition();
        robot.updatePositionInches();
        telemetry.addData("Left ticks", robot.getLeftTicks());
//        telemetry.addData("Center ticks", robot.getCenterTicks());
        telemetry.addData("Right ticks", robot.getRightTicks());
//        telemetry.addData("Left ticks", robot.getLTicksTelem());
//        telemetry.addData("Center ticks", robot.getCTicksTelem());
//        telemetry.addData("Right ticks", robot.getRTicksTelem());
        telemetry.addData("Left ticks direct", robot.LencoderDrive().getCurrentPosition());
        telemetry.addData("Right ticks direct", robot.RencoderDrive().getCurrentPosition());
        telemetry.addData("Center ticks direct", robot.CencoderDrive().getCurrentPosition());
        telemetry.addData("Left inches direct", (robot.LencoderDrive().getCurrentPosition() * RobotConstants.INCHES_PER_COUNT));
        telemetry.addData("Right inches direct", (robot.RencoderDrive().getCurrentPosition() * RobotConstants.INCHES_PER_COUNT));
        telemetry.addData("delta left", robot.getDeltaLeft());
        telemetry.addData("delta right", robot.getDeltaRight());
//        telemetry.addData("delta center", robot.getDeltaCenter());
//        telemetry.addData("delta left", robot.getDLeftTelem());
//        telemetry.addData("delta right", robot.getDRightTelem());
//        telemetry.addData("delta center", robot.getDCenterTelem());
        telemetry.addData("X value", robot.getX());
        telemetry.addData("Y value", robot.getY());
        telemetry.addData("Theta value (rad)", robot.getTheta());
        telemetry.addData("Theta value (deg)", robot.getThetadeg());
        telemetry.update();

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
        } else {
            robot.FLdrive().setPower(0);
            robot.FRdrive().setPower(0);
            robot.BLdrive().setPower(0);
            robot.BRdrive().setPower(0);
        }

       // robot.getLeftTicks();
        //robot.updatePositionNoReset();
        // telemetry.addData("Left ticks", robot.getLeftTicks());
        // telemetry.addData("Center ticks", robot.getCenterTicks());
        // telemetry.addData("Right ticks", robot.getRightTicks());
        // telemetry.addData("X value", robot.getX());
        // telemetry.addData("Y value", robot.getY());
        // telemetry.addData("Theta value", robot.getTheta());
        // telemetry.update();

    }

    @Override
    public void stop() {
    }
}
