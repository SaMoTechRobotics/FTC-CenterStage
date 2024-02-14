package org.firstinspires.ftc.teamcode.util.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.robot.arm.Arm;
import org.firstinspires.ftc.teamcode.util.robot.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.robot.claw.Claw;
import org.firstinspires.ftc.teamcode.util.vision.Vision;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;

public class Robot {
    private final Telemetry telemetry;

    public Vision vision;
    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public Boolean pickUp = true;
    public Boolean droneReady = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
//        vision = new Vision(hardwareMap);

        chassis = new Chassis(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }

    public void resetForIntake() {
        arm.setHangingLock(false);
        arm.setRotation(ArmRotation.Down);
        arm.setWristRotation(WristRotation.Down);
        claw.open();
    }

    public void prepareDroneLaunch() {
        arm.setRotation(ArmRotation.DroneLaunch);
        arm.setWristRotation(WristRotation.Up);
        droneReady = true;
    }

    public void update() {
        chassis.update();
        arm.update();
        pickUp = Math.floor(arm.getRotation()) < ArmRotation.MaxPickup;
    }
}
