package org.firstinspires.ftc.teamcode.util.robot.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.lib.MotorsSyncManager;
import org.firstinspires.ftc.teamcode.util.robot.claw.ClawPosition;

public class Arm {
    private final MotorsSyncManager armMotors;
    private final Servo wristServo;
    private final Servo droneServo;

    private boolean globalWristRotation = false;

    private int startingRotation = 0;

    private double boardAngle = WristRotation.DefaultBoardAngle;

    public Arm(HardwareMap hardwareMap) {
        DcMotor motor = hardwareMap.get(DcMotor.class, "arm0");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "arm1");
        armMotors = new MotorsSyncManager(motor, motor2);
        armMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo = hardwareMap.get(Servo.class, "wrist");
        droneServo = hardwareMap.get(Servo.class, "drone");

        setWristRotation(WristRotation.Down);
        droneServo.setPosition(ClawPosition.DroneLock);
    }

    public void update() {
        if (globalWristRotation) updateGlobalWristRotation();

        updateSpeed();
    }

    public void launchDrone() {
        droneServo.setPosition(ClawPosition.DroneRelease);
    }

    public void lockDrone() {
        droneServo.setPosition(ClawPosition.DroneLock);
    }

    public double getRotation() {
        return armMotors.getCurrentPosition() * (90.0 / ArmRotation.TicksAt90Degrees);
    }

    public double getArmTicks() {
        return armMotors.getCurrentPosition();
    }

    public double getWristRotation() {
        return wristServo.getPosition();
    }

    public void setWristPickup(Boolean value) {
        if (value) {
            setRotation(ArmRotation.Down);
            setWristRotation(WristRotation.Down);
        } else {
            setRotation(ArmRotation.HoldDown);
            setWristRotation(WristRotation.HoldDown);
        }
    }

    public void holdRotation() {
        if (armMotors.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            armMotors.setTargetPosition(armMotors.getCurrentPosition());
            armMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotors.setPower(ArmSpeed.HoldSpeed);
        }
    }

    public void updateSpeed() {
        if (armMotors.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return;
        double conversion = (90.0 / ArmRotation.TicksAt90Degrees);
        double current = armMotors.getCurrentPosition() * conversion;
        double target = armMotors.getTargetPosition() * conversion;
        // make it get to max speed in ArmRotation.RampUpAngle degrees and then go down when at target - ArmRotation.RampUpAngle
        double diffToTarget = Math.abs(target - current);
        double diffToStart = Math.abs(current - startingRotation);
//        if (diffToStart < ArmRotation.RampUpAngle) {
//            armMotors.setPower((ArmSpeed.Max - ArmSpeed.Min) * (diffToStart / ArmRotation.RampUpAngle) + ArmSpeed.Min);
        if (diffToTarget < ArmRotation.RampUpAngle) {
            armMotors.setPower((ArmSpeed.Max - ArmSpeed.MinSlowingDown) * (diffToTarget / ArmRotation.RampUpAngle) + ArmSpeed.MinSlowingDown);
        } else {
            armMotors.setPower(ArmSpeed.Max);
        }
    }

    public void setRotation(double degrees) {
        armMotors.setTargetPosition(degreesToArmTicks(degrees));
        startingRotation = armMotors.getCurrentPosition();
        armMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double diff = Math.abs(degrees - armMotors.getCurrentPosition() * 90.0 / ArmRotation.TicksAt90Degrees);
        if (diff < 15) {
            armMotors.setPower(ArmSpeed.Min);
        } else if (diff < 45) {
            armMotors.setPower(ArmSpeed.Mid);
        } else {
            armMotors.setPower(ArmSpeed.Max);
        }
    }

    public void setRotation(double degrees, double speed) {
        armMotors.setTargetPosition(degreesToArmTicks(degrees));
        armMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotors.setPower(speed);
    }

    public void manualRotation(double power) {
        armMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotors.setPower(power);
    }

    public void setGlobalWristRotation(boolean value) {
        globalWristRotation = value;
    }

    public void updateGlobalWristRotation() {
        int ticks = armMotors.getCurrentPosition();
        if (armMotors.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ticks = armMotors.getTargetPosition();
        double rotation = ticks * (90.0 / ArmRotation.TicksAt90Degrees);
        double wristRotation;
        if (rotation < ArmRotation.MaxPickup) {
            wristRotation = WristRotation.PickupComplementAngle - rotation;
        } else {
            wristRotation = rotation - boardAngle;
        }
        double wristPosition = degreesToWristPosition(wristRotation);
        wristServo.setPosition(wristPosition);
    }

    public void setBoardAngle(double angle) {
        boardAngle = angle;
    }

    public boolean getGlobalWristRotationEnabled() {
        return globalWristRotation;
    }

    public void setWristRotation(double position) {
        wristServo.setPosition(position);
        globalWristRotation = false;
    }

    public void setHangingLock(boolean lock) {
        if (lock) {
            armMotors.setTargetPosition(degreesToArmTicks(ArmRotation.HangingLock));
            armMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristServo.setPosition(WristRotation.HangLock);
            armMotors.setPower(ArmSpeed.Max);
        } else {
            armMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotors.setPower(0);
        }
    }

    public void testWristDegrees(double degrees, Telemetry telemetry) {
        wristServo.setPosition(degreesToWristPosition(degrees));
        telemetry.addData("Wrist Position At " + degrees + " deg", degreesToWristPosition(degrees));
    }

    private int degreesToArmTicks(double degrees) {
        return (int) (degrees * (ArmRotation.TicksAt90Degrees / 90.0));
    }

    private double degreesToWristPosition(double degrees) {
        return (degrees * (Math.abs(WristRotation.PositionAt90Degrees - WristRotation.PositionAt0Degrees)) / 90.0 + WristRotation.PositionAt0Degrees);
    }
}
