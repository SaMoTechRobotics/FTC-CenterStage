package org.firstinspires.ftc.teamcode.util.robot.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class ChassisSpeed {

    public static PoseVelocity2d Max = new PoseVelocity2d(
            new Vector2d(1.0, 1.0),
            1.0
    );
    public static PoseVelocity2d Mid = new PoseVelocity2d(
            new Vector2d(0.5, 0.5),
            0.5
    );
    public static PoseVelocity2d Min = new PoseVelocity2d(
            new Vector2d(0.2, 0.2),
            0.25
    );

    /**
     * The max and min speeds of the chassis motors
     */
    //    public static final double Max = 1.0;
    //    public static final double Min = 0.0;
    /**
     * The speed of the robot when it is moving forward or backward
     */
    public static double MaxDrive = 1.0;
    public static double MidDrive = 0.5;
    public static double MinDrive = 0.2;
    /**
     * The speed of the robot when it is turning
     */
    public static double MaxTurn = 1.0;
    public static double MidTurn = 0.5;
    public static double MinTurn = 0.2;
    /**
     * The speed of the robot when it is strafing
     */
    public static double MaxStrafe = 1.0;
    public static double MidStrafe = 0.5;
    public static double MinStrafe = 0.25;

    public static double MatchingArmSpeed = 0.0; // 0.3
    public static double MatchingArmSlowSpeed = 0.0; // 0.15

    public static double JoystickYMargin = 0.1;
    public static double JoystickXMargin = 0.15;

    public static double BoardAlignmentSpeed = 0.2;
}
