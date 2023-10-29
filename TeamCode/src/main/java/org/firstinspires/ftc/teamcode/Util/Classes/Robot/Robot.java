package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

public class Robot {
    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public Boolean pickUp = true;

    public Robot(HardwareMap hardwareMap) {
        chassis = new Chassis(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    public void resetForIntake() {
        arm.setHangingLock(false);
        arm.setRotation(ArmRotation.Down);
        arm.setWristRotation(WristRotation.Down);
        claw.setOpen(true);
    }

    public void prepareDroneLaunch() {
        arm.setRotation(ArmRotation.DroneLaunch);
        arm.setWristRotation(WristRotation.Down);
        claw.setOpen(true);
    }

    public void update() {
        arm.update();
        if (arm.getRotation() < ArmRotation.MaxPickup) {
            pickUp = true;
        } else {
            pickUp = false;
        }
    }
}
