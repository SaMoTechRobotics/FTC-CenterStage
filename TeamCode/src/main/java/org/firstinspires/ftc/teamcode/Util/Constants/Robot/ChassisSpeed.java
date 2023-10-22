package org.firstinspires.ftc.teamcode.Util.Constants.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ChassisSpeed {
    /**
     * The max and min speeds of the chassis motors
     */
    public static final double Max = 1.0;
    public static final double Min = 0.0;
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
    /**
     * Place speed for when robot is aligning with pole
     */
    public static double AlignSpeed = 0.16;
    public static double FineAlignSpeed = 0.07;

    public static double ManualAlignSpeed = 0.2;

    public static double QuickPlaceSpeed = 20;

    public static double QuickPlaceAccel = 10;

    /**
     * Place speed for when robot is driving toward pole to place cone
     */
    public static double PlaceSpeed = 0.15;
    public static double ManualPlaceSpeed = 0.2;

    public static double JoystickYMargin = 0.1;
    public static double JoystickXMargin = 0.15;

}
