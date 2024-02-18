package org.firstinspires.ftc.teamcode.util.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.robot.arm.Arm;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;
import org.firstinspires.ftc.teamcode.util.robot.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.robot.claw.Claw;
import org.firstinspires.ftc.teamcode.util.vision.Vision;

public class Robot {
    public Vision vision;
    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public boolean pickUp = true;

    public boolean wristLevelingEnabled = true;

    public Robot(HardwareMap hardwareMap) {
//        vision = new Vision(hardwareMap);

        chassis = new Chassis(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    public void resetForIntake() {
        arm.setHangingLock(false);
        arm.setRotation(ArmRotation.Down);
        arm.setWristRotation(WristRotation.Down);
        claw.open();
    }

    public void update() {
        chassis.update();
        arm.update();
        pickUp = Math.floor(arm.getRotation()) < ArmRotation.MaxPickup;
    }
}
