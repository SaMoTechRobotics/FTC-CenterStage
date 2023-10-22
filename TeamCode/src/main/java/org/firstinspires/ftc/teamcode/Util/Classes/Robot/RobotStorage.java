package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotStorage {
    public static Pose2d pose = new Pose2d(0, 0, 0);

    public static void reset() {
        pose = new Pose2d(0, 0, 0);
    }
}
