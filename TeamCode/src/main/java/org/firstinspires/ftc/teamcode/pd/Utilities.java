package org.firstinspires.ftc.teamcode.pd;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Utilities {
    public static double deltaLeftDistance = 0;
    public static double deltaRightDistance = 0;
    public static double deltaCenterDistance = 0;
    public static double x = 0;
    public static double y = 0;
    public static double theta = 0;

    static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    static final double oneRotationTicks = 800;
    static final double wheelRadius = 0.025; // in meters (0.05)
    static final double wheelDistanceApart = 0.345; // in meters (the right and left odo wheels are .167 m from the center one)
    static final double wheelDistanceApartInches = 13.583;

    public static double thetadeg;
    public static double LticksTelem;
    public static double RticksTelem;
    public static double CticksTelem;
    public static double DLeftTelem;
    public static double DRightTelem;
    public static double DCenterTelem;

    public static void updatePosition() {
        deltaLeftDistance = (100 / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;      // OKAY SO WHAT TO DO IS USE CALCULATIONS FROM
        deltaRightDistance = (100 / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;    // SIMPLE ODO AUTO AND ROBOT CONTSTANTS TO FIND
        deltaCenterDistance = (100 / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;  // THE NEW DELTA L AND R DISTANCES
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (((deltaLeftDistance - deltaRightDistance) / wheelDistanceApart)); // added a * .15 here to correct the theta
        thetadeg  += (((theta / 2) / Math.PI) * 360);
        LticksTelem += 100;
        RticksTelem += 100;
        CticksTelem += 100;
        DLeftTelem  += deltaLeftDistance;
        DRightTelem += deltaRightDistance;
        DCenterTelem += deltaCenterDistance;
    }
}
