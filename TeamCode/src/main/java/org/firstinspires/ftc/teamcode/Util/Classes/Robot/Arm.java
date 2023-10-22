package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Util.Constants.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.WristRotation;

public class Arm {
    private final DcMotor armMotor;
    private final Servo wristServo;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void setArmRotation(double degrees) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
    }

    public void setWristRotation(double degrees) {
        wristServo.setPosition(degreesToWristPosition(degrees));
    }

    private int degreesToArmTicks(double degrees) {
        return (int) (degrees * (ArmRotation.TicksAt90Degrees / 90.0));
    }

    private int degreesToWristPosition(double degrees) {
        return (int) (degrees * (WristRotation.PositionAt180Degrees / 180.0));
    }
}
