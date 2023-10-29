package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

public class Arm {
    private final DcMotor armMotor;
    private final Servo wristServo;

    private boolean globalWristRotation = false;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void update(Telemetry telemetry) {
        if (globalWristRotation) updateGlobalWristRotation(telemetry);

    }

    public void setRotation(double degrees) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ArmSpeed.MaxSpeed);
    }

    public void manualRotation(double power) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    public void setGlobalWristRotation(boolean value) {
        globalWristRotation = value;
    }

    public void updateGlobalWristRotation(Telemetry telemetry) {
        telemetry.addLine("Updating... " + Math.random());
        int ticks = armMotor.getCurrentPosition();
//        telemetry.addData("Arm Position", ticks);
        double rotation = ticks * (90.0 / ArmRotation.TicksAt90Degrees);
//        telemetry.addData("Arm Rotation", rotation);
        double wristRotation = rotation - WristRotation.ArmComplementAngle;
//        telemetry.addData("Wrist Rotation", wristRotation);
        double wristPosition = degreesToWristPosition(wristRotation);
//        telemetry.addData("Wrist Position", wristPosition);
        wristServo.setPosition(wristPosition);
    }

    public void setWristRotation(double position) {
        wristServo.setPosition(position);
        globalWristRotation = false;
    }

    public void setHangingLock(boolean lock) {
        if (lock) {
            armMotor.setTargetPosition(degreesToArmTicks(ArmRotation.HangingLock));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristServo.setPosition(WristRotation.Hang);
            armMotor.setPower(ArmSpeed.MaxSpeed);
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
        return (degrees * (Math.abs(WristRotation.PositionAt180Degrees - WristRotation.PositionAt0Degrees)) / 180.0 + WristRotation.PositionAt0Degrees);
    }
}
