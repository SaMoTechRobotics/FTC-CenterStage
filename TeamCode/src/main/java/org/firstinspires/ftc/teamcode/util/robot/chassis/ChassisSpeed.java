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

    public static double DriveDeadZone = 0.1;
    public static double StrafeDeadZone = 0.1;
    public static double TurnDeadZone = 0.1;

    public static double applyDeadZone(double val, double m) {
        return Math.abs(val) > m ? val : 0;
    }

    public static double BoardAlignmentSpeed = 0.2;
}
