package org.firstinspires.ftc.teamcode.Util.Classes.Storage;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.FieldStartingPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

public class RobotStorage {
    public static Pose2d pose = new Pose2d(0, 0, 0);

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
            x = side == AutoSide.NEAR ? FieldStartingPosition.RedFarX : FieldStartingPosition.RedNearX;
        }
        double y = color == AutoColor.BLUE ? FieldStartingPosition.BlueY : FieldStartingPosition.RedY;
        double rot = color == AutoColor.BLUE ? FieldStartingPosition.BlueRot : FieldStartingPosition.RedRot;
        return new Pose2d(x, y, Math.toRadians(rot));
    }
}
