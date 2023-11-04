package org.firstinspires.ftc.teamcode.Auto.Base;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.DistanceSensorConstants;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;

@Config
public class BaseAuto extends LinearOpMode {
    private static Robot robot;

    public static AutoSide SIDE = AutoSide.RIGHT;
    public static AutoColor COLOR = AutoColor.RED;

    protected static class DriveDistances {
        public static Double ToParking = 50.0;
    }

    protected void initAuto() {
        RobotStorage.reset(SIDE, COLOR);
        robot = new Robot(hardwareMap);
    }

    @Override
    public void runOpMode() {
        while (!isStarted()) {
        }

        waitForStart();
        if (isStopRequested()) return;
    }

    private SpikeLocation detectSpikeLocation() {
        double leftDistance = robot.chassis.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        double rightDistance = robot.chassis.rightDistanceSensor.getDistance(DistanceUnit.INCH);

        if (leftDistance < DistanceSensorConstants.SpikeMarkDistance) {
            return SIDE == AutoSide.RIGHT ? SpikeLocation.CENTER : SpikeLocation.LEFT;
        } else if (rightDistance < DistanceSensorConstants.SpikeMarkDistance) {
            return SIDE == AutoSide.RIGHT ? SpikeLocation.RIGHT : SpikeLocation.CENTER;
        } else {
            return SIDE == AutoSide.RIGHT ? SpikeLocation.LEFT : SpikeLocation.RIGHT;
        }
    }
}

