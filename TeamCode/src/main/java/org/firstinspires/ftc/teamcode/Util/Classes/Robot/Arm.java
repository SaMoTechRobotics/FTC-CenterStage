package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

public class Arm {
    private final DcMotor armMotor;
    private final Servo wristServo;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void setRotation(double degrees) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ArmSpeed.MaxSpeed);
    }

    public void setGlobalWristRotation(double degrees) {
        int ticks = armMotor.getCurrentPosition();
        double rotation = ticks / (ArmRotation.TicksAt90Degrees / 90.0);
        setWristRotation(degrees - rotation);
    }

    public void setWristRotation(double degrees) {
        wristServo.setPosition(degreesToWristPosition(degrees));
    }

    public void setHangingLock(boolean lock) {
        if (lock) {
            armMotor.setTargetPosition(degreesToArmTicks(ArmRotation.Hang));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristServo.setPosition(degreesToWristPosition(WristRotation.Hang));
            armMotor.setPower(ArmSpeed.MaxSpeed);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0);
        }
    }

    private int degreesToArmTicks(double degrees) {
        return (int) (degrees * (ArmRotation.TicksAt90Degrees / 90.0));
    }

    private int degreesToWristPosition(double degrees) {
        return (int) (degrees * (WristRotation.PositionAt180Degrees / 180.0));
    }
}
