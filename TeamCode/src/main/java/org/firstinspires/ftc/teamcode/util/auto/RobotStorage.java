package org.firstinspires.ftc.teamcode.util.auto;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.util.auto.constants.FieldStartingPosition;

public class RobotStorage {
    public static Pose2d pose = new Pose2d(0, 0, 0);

    public static boolean armUp = false;

    public static void reset() {
        pose = new Pose2d(0, 0, 0);
    }

    public static void setPose(Pose2d newPose) {
        pose = newPose;
    }

    public static Pose2d getStartPose(AutoSide side, AutoColor color) {
        double x;
        if (color == AutoColor.BLUE) {
            x = side == AutoSide.NEAR ? FieldStartingPosition.BlueNearX : FieldStartingPosition.BlueFarX;
        } else {
            x = side == AutoSide.NEAR ? FieldStartingPosition.RedNearX : FieldStartingPosition.RedFarX;
        }
        double y = color == AutoColor.BLUE ? FieldStartingPosition.BlueY : FieldStartingPosition.RedY;
        double rot = color == AutoColor.BLUE ? FieldStartingPosition.BlueRot : FieldStartingPosition.RedRot;
        return new Pose2d(x, y, Math.toRadians(rot));
    }
}
