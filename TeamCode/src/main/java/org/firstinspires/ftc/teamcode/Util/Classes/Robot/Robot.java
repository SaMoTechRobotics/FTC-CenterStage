package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

public class Robot {
    private final Telemetry telemetry;

    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public Boolean pickUp = true;
    public Boolean droneReady = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        chassis = new Chassis(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }

    public void resetForIntake() {
        arm.setHangingLock(false);
        arm.setRotation(ArmRotation.Down);
        arm.setWristRotation(WristRotation.Down);
        claw.setOpen(true);
    }

    public void prepareDroneLaunch() {
        arm.setRotation(ArmRotation.DroneLaunch);
        arm.setWristRotation(WristRotation.Up);
        claw.setOpen(true);
        droneReady = true;
    }

    public void update() {
        arm.update();
        pickUp = arm.getRotation() < ArmRotation.MaxPickup;
    }
}
