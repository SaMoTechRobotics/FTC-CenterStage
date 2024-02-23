package org.firstinspires.ftc.teamcode.util.robot.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;

@Config
public class Chassis {
    public static boolean headingFixEnabled = true;

    private final MecanumDrive drive;

    private final IMU imu;

    private PoseVelocity2d speed = ChassisSpeed.Mid;

    private boolean lockHeading = false;
    private double startHeading = 0;

    public Chassis(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, RobotStorage.pose);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public void update() {
        drive.updatePoseEstimate();
    }

    public void updateSpeed(boolean high, boolean low) {
        if (high) speed = ChassisSpeed.Max;
        else if (low) speed = ChassisSpeed.Min;
        else speed = ChassisSpeed.Mid;
    }

    public void setManualPower(double drivePower, double strafePower, double turnPower) {
        if (turnPower != 0) { // If started turning
            lockHeading = false;
            startHeading = getCurrentHeadingRadians(); // Get current heading from imu, this is the heading that will be locked for strafing or driving
        } else if (!lockHeading && (drivePower != 0 || strafePower != 0)) { // If not already starting holding heading and robot is moving without rotation
            lockHeading = true;
            startHeading = getCurrentHeadingRadians(); // Get current heading from imu, this is the heading that will be locked for strafing or driving
        } else if (lockHeading && drivePower == 0 && strafePower == 0) { // If heading is locked but should not be anymore
            lockHeading = false;
        }

        // If heading lock is enabled
        if (headingFixEnabled && speed == ChassisSpeed.Min && lockHeading) {

            double headingError = getCurrentHeadingRadians() - startHeading; // The current heading from imu - the starting (target) heading

            // Which way to rotate to correct
            if (headingError > Math.PI) headingError -= Math.PI;
            else if (headingError < -Math.PI) headingError += Math.PI;

            // How fast to rotate to correct
            double headingCorrectionPower = -MecanumDrive.PARAMS.headingCorrectionCoefficient * headingError;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -drivePower * speed.linearVel.x,
                            -strafePower * speed.linearVel.y
                    ),
                    headingCorrectionPower
            ));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -drivePower * speed.linearVel.x,
                            -strafePower * speed.linearVel.y
                    ),
                    -turnPower * speed.angVel
            ));
        }
    }

    private double getCurrentHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getCurrentHeadingDegrees() {
        return Math.toDegrees(getCurrentHeadingRadians());
    }

    public Pose2d getPose() {
        return drive.pose;
    }
}