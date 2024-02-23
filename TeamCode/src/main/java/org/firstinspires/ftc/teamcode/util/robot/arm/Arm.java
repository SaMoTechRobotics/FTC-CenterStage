package org.firstinspires.ftc.teamcode.util.robot.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.robot.claw.ClawPosition;

public class Arm {
    private final DcMotor armMotor;
    private final Servo wristServo;
    private final Servo droneServo;

    private boolean globalWristRotation = false;

    private double boardAngle = WristRotation.DefaultBoardAngle;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristServo = hardwareMap.get(Servo.class, "wrist");
        droneServo = hardwareMap.get(Servo.class, "drone");
        setWristRotation(WristRotation.Down);
        droneServo.setPosition(ClawPosition.DroneLock);

    }

    public void update() {
        if (globalWristRotation) updateGlobalWristRotation();
    }

    public void launchDrone() {
        droneServo.setPosition(ClawPosition.DroneRelease);
    }

    public void lockDrone() {
        droneServo.setPosition(ClawPosition.DroneLock);
    }

    public double getRotation() {
        return armMotor.getCurrentPosition() * (90.0 / ArmRotation.TicksAt90Degrees);
    }

    public double getArmTicks() {
        return armMotor.getCurrentPosition();
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
        if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ArmSpeed.HoldSpeed);
        }
    }

    public void setRotation(double degrees) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double diff = Math.abs(degrees - armMotor.getCurrentPosition() * 90.0 / ArmRotation.TicksAt90Degrees);
        if (diff < 15) {
            armMotor.setPower(ArmSpeed.Min);
        } else if (diff < 45) {
            armMotor.setPower(ArmSpeed.Mid);
        } else {
            armMotor.setPower(ArmSpeed.Max);
        }
    }

    public void setRotation(double degrees, double speed) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(speed);
    }

    public void manualRotation(double power) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    public void setGlobalWristRotation(boolean value) {
        globalWristRotation = value;
    }

    public void updateGlobalWristRotation() {
        int ticks = armMotor.getCurrentPosition();
        if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ticks = armMotor.getTargetPosition();
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
            armMotor.setTargetPosition(degreesToArmTicks(ArmRotation.HangingLock));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristServo.setPosition(WristRotation.HangLock);
            armMotor.setPower(ArmSpeed.Max);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0);
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
