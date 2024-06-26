package org.firstinspires.ftc.teamcode.util.robot.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WristRotation {
    public static double DefaultBoardAngle = 110.0; // 105.0
    public static double AutoBoardAngle = 102.0; // 105.0
    public static double PickupComplementAngle = 180.0;

    public static double MinHighBoardAngle = 50.0; // 105.0
    public static double MaxHighBoardAngle = 110.0; // 105.0
    public static double HighCoefficient = 2;

    public static double PositionAt0Degrees = 0.34; // 0.3
    public static double PositionAt90Degrees = 0.05; //0.8

    public static double HoldDown = 0.91;
    public static double Down = 0.88;
    public static double StackDown = 0.9;
    public static double Up = 1.0;
    public static double Forward = 0.5;

    public static double PickupBack = 0.3;

    public static double Hang = 0.7;
    public static double HangLock = 0.95;

    public static double AutoPickupStack = 0.9;
}
