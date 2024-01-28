package org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Classes.Storage.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.DistanceSensorConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ChassisSpeed;

@Config
public class Chassis {
    public static boolean headingFix = false;

    private final Telemetry telemetry;

    MecanumDrive drive;

    IMU imu;

    public Wheels wheels;

    private PoseVelocity2d speed = ChassisSpeed.Mid;

    public double driveSpeed = ChassisSpeed.MidDrive;
    public double turnSpeed = ChassisSpeed.MidTurn;
    public double strafeSpeed = ChassisSpeed.MidStrafe;
    public boolean brake = true;

    private boolean lockHeading = false;
    private double startHeading = 0;

    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.wheels =
                new Wheels(
                        hardwareMap.get(DcMotor.class, "frontLeft"),
                        hardwareMap.get(DcMotor.class, "frontRight"),
                        hardwareMap.get(DcMotor.class, "backLeft"),
                        hardwareMap.get(DcMotor.class, "backRight")
                );

        this.wheels.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.wheels.backLeft.setDirection(DcMotor.Direction.REVERSE);

        this.leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "dist3");
        this.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "dist2");

        drive = new MecanumDrive(hardwareMap, RobotStorage.pose);

        imu = hardwareMap.get(IMU.class, "imu");

        toggleBrake(true);
    }

    public void update() {
        drive.updatePoseEstimate();

        if (Drive.DebuggingTelemetry) {
            Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
            dashboardTelemetry.addData("ld", Math.min(leftDistanceSensor.getDistance(DistanceUnit.INCH), 20));
            dashboardTelemetry.addData("rd", Math.min(rightDistanceSensor.getDistance(DistanceUnit.INCH), 20));
            dashboardTelemetry.addData("Pixel Detected In Claw", pixelDetectedInClaw());
            dashboardTelemetry.addData("Pixel Detected", pixelDetected());
            dashboardTelemetry.update();
            telemetry.addData("Left Distance", leftDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance", rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Pixel Detected In Claw", pixelDetectedInClaw());
            telemetry.addData("Pixel Detected", pixelDetected());
        }
    }

    public boolean pixelDetectedInClaw() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) < DistanceSensorConstants.PixelInClawMaxDistance &&
                rightDistanceSensor.getDistance(DistanceUnit.INCH) < DistanceSensorConstants.PixelInClawMaxDistance;
    }

    public boolean pixelDetected() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) < DistanceSensorConstants.PixelDetectedDistance &&
                rightDistanceSensor.getDistance(DistanceUnit.INCH) < DistanceSensorConstants.PixelDetectedDistance;
    }

    public void alignWithPixel() {
        double ld = leftDistanceSensor.getDistance(DistanceUnit.INCH);
        double rd = rightDistanceSensor.getDistance(DistanceUnit.INCH);

        PoseVelocity2d alignSpeed = ChassisSpeed.Min;

        if (isCloseEnough(ld, rd, DistanceSensorConstants.PixelCenteredMargin)) {
            if (!pixelDetectedInClaw()) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, alignSpeed.linearVel.y),
                                0
                        )
                );
            } else {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, 0),
                                0
                        )
                );
            }
        } else if (ld < rd) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(alignSpeed.linearVel.x, 0),
                            0
                    )
            );
        } else {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-alignSpeed.linearVel.x, 0),
                            0
                    )
            );
        }
    }

    private boolean isCloseEnough(double a, double b, double margin) {
        return Math.abs(a - b) < margin;
    }

    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    public void toggleBrake(boolean brakeOn) {
        this.brake = brakeOn;
        DcMotor.ZeroPowerBehavior behavior = brakeOn
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        this.wheels.frontLeft.setZeroPowerBehavior(behavior);
        this.wheels.frontRight.setZeroPowerBehavior(behavior);
        this.wheels.backLeft.setZeroPowerBehavior(behavior);
        this.wheels.backRight.setZeroPowerBehavior(behavior);
    }

    public void updateSpeed(boolean high, boolean low) {
        if (high) speed = ChassisSpeed.Max;
        else if (low) speed = ChassisSpeed.Min;
        else speed = ChassisSpeed.Mid;
//        if (high) { // Left bumper is max speeds
//            this.driveSpeed = ChassisSpeed.MaxDrive;
//            this.turnSpeed = ChassisSpeed.MaxTurn;
//            this.strafeSpeed = ChassisSpeed.MaxStrafe;
//            this.toggleBrake(true);
//        } else if (low) { // Right bumper is min speeds
//            this.driveSpeed = ChassisSpeed.MinDrive;
//            this.turnSpeed = ChassisSpeed.MinTurn;
//            this.strafeSpeed = ChassisSpeed.MinStrafe;
//            this.toggleBrake(true);
//        } else { // No bumper is mid speeds
//            this.driveSpeed = ChassisSpeed.MidDrive;
//            this.turnSpeed = ChassisSpeed.MidTurn;
//            this.strafeSpeed = ChassisSpeed.MidStrafe;
//            this.toggleBrake(true);
//        }
    }
//
//    public void updateWithControls(
//            GamepadEx gamepad1,
//            GamepadEx gamepad2,
//            boolean delivering
//    ) {
//        double driveStick = Math.abs(gamepad1.getLeftY()) > ChassisSpeed.JoystickYMargin ? gamepad1.getLeftY() : 0;
//        if (delivering) {
//            if (Math.abs(gamepad2.getRightX()) > 0.01) {
//                driveStick += gamepad2.getLeftX() * ChassisSpeed.MatchingArmSlowSpeed;
//            } else if (Math.abs(gamepad2.getLeftX()) > 0.01) {
//                driveStick += gamepad2.getLeftX() * ChassisSpeed.MatchingArmSpeed;
//            }
//        }
//        double strafeStick = Math.abs(gamepad1.getLeftX()) > ChassisSpeed.JoystickXMargin ? gamepad1.getLeftX() : 0;
//        double turnStick = gamepad1.getRightX();
//        if (driveStick != 0 || strafeStick != 0 || turnStick != 0) {
//            this.setManualPower(driveStick, strafeStick, turnStick);
//        } else {
//            this.setPower(this.wheels.frontLeft, 0);
//            this.setPower(this.wheels.frontRight, 0);
//            this.setPower(this.wheels.backLeft, 0);
//            this.setPower(this.wheels.backRight, 0);
//        }
//    }

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
        if (headingFix && lockHeading) {

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

    public void setPowerAllMotors(Double speed) {
        this.setPower(this.wheels.frontLeft, speed);
        this.setPower(this.wheels.frontRight, speed);
        this.setPower(this.wheels.backRight, speed);
        this.setPower(this.wheels.backLeft, speed);
    }

    public Pose2d getPose() {
        return drive.pose;
    }


    private static class Wheels {
        public DcMotor frontLeft;
        public DcMotor frontRight;
        public DcMotor backLeft;
        public DcMotor backRight;

        public Wheels(
                DcMotor frontLeft,
                DcMotor frontRight,
                DcMotor backLeft,
                DcMotor backRight
        ) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }
    }
}
