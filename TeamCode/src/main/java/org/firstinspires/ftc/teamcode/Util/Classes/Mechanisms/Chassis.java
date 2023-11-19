package org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive.Drive;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.DistanceSensorConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ChassisSpeed;

public class Chassis {
    private final Telemetry telemetry;

    public Wheels Wheels;
    public double DriveSpeed = ChassisSpeed.MidDrive;
    public double TurnSpeed = ChassisSpeed.MidTurn;
    public double StrafeSpeed = ChassisSpeed.MidStrafe;
    public boolean brake = true;

    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.Wheels =
                new Wheels(
                        hardwareMap.get(DcMotor.class, "frontLeft"),
                        hardwareMap.get(DcMotor.class, "frontRight"),
                        hardwareMap.get(DcMotor.class, "backLeft"),
                        hardwareMap.get(DcMotor.class, "backRight")
                );

        this.Wheels.FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.Wheels.BackLeft.setDirection(DcMotor.Direction.REVERSE);

        this.leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistance");
        this.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistance");

        toggleBrake(true);
    }

    public void update() {
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

    /**
     * Sets the power of the motor provided
     *
     * @param motor The motor to set the power of
     * @param power The power to set the motors to, from -1.0 to 1.0
     */
    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    /**
     * Toggles the zero power behavior of the motors
     *
     * @param brakeOn Whether or not to brake on zero power
     */
    public void toggleBrake(boolean brakeOn) {
        this.brake = brakeOn;
        DcMotor.ZeroPowerBehavior behavior = brakeOn
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        this.Wheels.FrontLeft.setZeroPowerBehavior(behavior);
        this.Wheels.FrontRight.setZeroPowerBehavior(behavior);
        this.Wheels.BackLeft.setZeroPowerBehavior(behavior);
        this.Wheels.BackRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets the speed of the chassis based of the gamepad1 bumpers
     *
     * @param high Whether or not the high speed button is pressed, gamepad1 left bumper
     * @param low  Whether or not the low speed button is pressed, gamepad1 right bumper
     */
    public void updateSpeed(boolean high, boolean low) {
        if (high) { // Left bumper is max speeds
            this.DriveSpeed = ChassisSpeed.MaxDrive;
            this.TurnSpeed = ChassisSpeed.MaxTurn;
            this.StrafeSpeed = ChassisSpeed.MaxStrafe;
            this.toggleBrake(true);
        } else if (low) { // Right bumper is min speeds
            this.DriveSpeed = ChassisSpeed.MinDrive;
            this.TurnSpeed = ChassisSpeed.MinTurn;
            this.StrafeSpeed = ChassisSpeed.MinStrafe;
            this.toggleBrake(true);
        } else { // No bumper is mid speeds
            this.DriveSpeed = ChassisSpeed.MidDrive;
            this.TurnSpeed = ChassisSpeed.MidTurn;
            this.StrafeSpeed = ChassisSpeed.MidStrafe;
            this.toggleBrake(true);
        }
    }

    public void updateWithControls(
            GamepadEx gamepad1,
            GamepadEx gamepad2,
            boolean delivering
    ) {
        double driveStick = Math.abs(-gamepad1.getLeftY()) > ChassisSpeed.JoystickYMargin ? -gamepad1.getLeftY() : 0;
        if (delivering) {
            if (Math.abs(gamepad2.getRightX()) > 0.01) {
                driveStick += gamepad2.getLeftX() * ChassisSpeed.MatchingArmSlowSpeed;
            } else if (Math.abs(gamepad2.getLeftX()) > 0.01) {
                driveStick += gamepad2.getLeftX() * ChassisSpeed.MatchingArmSpeed;
            }
        }
        double strafeStick = Math.abs(-gamepad1.getLeftX()) > ChassisSpeed.JoystickXMargin ? -gamepad1.getLeftX() : 0;
        double turnStick = gamepad1.getRightX();
        if (driveStick != 0 || strafeStick != 0 || turnStick != 0) {
            this.setManualPower(driveStick, strafeStick, turnStick);
        } else {
            this.setPower(this.Wheels.FrontLeft, 0);
            this.setPower(this.Wheels.FrontRight, 0);
            this.setPower(this.Wheels.BackLeft, 0);
            this.setPower(this.Wheels.BackRight, 0);
        }
    }

    private void setManualPower(double driveStick, double strafeStick, double turnStick) {
        /*
         * The different powers of the motors based of the joysticks
         */
        double frontLeftPower =
                (driveStick * this.DriveSpeed) +
                        (turnStick * this.TurnSpeed) +
                        (strafeStick * this.StrafeSpeed);
        double frontRightPower =
                (driveStick * this.DriveSpeed) -
                        (turnStick * this.TurnSpeed) -
                        (strafeStick * this.StrafeSpeed);
        double backLeftPower =
                (driveStick * this.DriveSpeed) +
                        (turnStick * this.TurnSpeed) -
                        (strafeStick * this.StrafeSpeed);
        double backRightPower =
                (driveStick * this.DriveSpeed) -
                        (turnStick * this.TurnSpeed) +
                        (strafeStick * this.StrafeSpeed);

        /*
         * Sets the power of all the motors for manual drive
         */
        this.setPower(this.Wheels.FrontLeft, frontLeftPower);
        this.setPower(this.Wheels.FrontRight, frontRightPower);
        this.setPower(this.Wheels.BackLeft, backLeftPower);
        this.setPower(this.Wheels.BackRight, backRightPower);

    }

    public void setPowerAllMotors(Double speed) {
        this.setPower(this.Wheels.FrontLeft, speed);
        this.setPower(this.Wheels.FrontRight, speed);
        this.setPower(this.Wheels.BackRight, speed);
        this.setPower(this.Wheels.BackLeft, speed);
    }

    /**
     * The motors of the chassis
     */
    public static class Wheels {
        public DcMotor FrontLeft;
        public DcMotor FrontRight;
        public DcMotor BackLeft;
        public DcMotor BackRight;

        /**
         * Constructor for the wheels
         *
         * @param frontLeft  The front left motor
         * @param frontRight The front right motor
         * @param backLeft   The back left motor
         * @param backRight  The back right motor
         */
        public Wheels(
                DcMotor frontLeft,
                DcMotor frontRight,
                DcMotor backLeft,
                DcMotor backRight
        ) {
            this.FrontLeft = frontLeft;
            this.FrontRight = frontRight;
            this.BackLeft = backLeft;
            this.BackRight = backRight;
        }
    }
}
