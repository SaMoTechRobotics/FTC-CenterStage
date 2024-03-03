package org.firstinspires.ftc.teamcode.util.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.robot.arm.Arm;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;
import org.firstinspires.ftc.teamcode.util.robot.claw.Claw;
import org.firstinspires.ftc.teamcode.util.vision.Vision;

public class AutoRobot {
    public final Vision vision;

    public final MecanumDrive drive;

    public final Arm arm;
    public final Claw claw;

    public AutoRobot(HardwareMap hardwareMap, Pose2d pose) {
        vision = new Vision(hardwareMap);

        drive = new MecanumDrive(hardwareMap, pose);

        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    public void resetForIntake() {
        arm.setHangingLock(false);
        arm.setRotation(ArmRotation.Down);
        arm.setWristRotation(WristRotation.Down);
        claw.open();
    }

    public Action openNextClaw() {
        return packet -> {
            claw.openNext();
            return false;
        };
    }

    public Action raiseArmForStack() {
        return packet -> {
            arm.setRotation(ArmRotation.StackAuto);
            return false;
        };
    }

    public Action prepareForDelivery() {
        return packet -> {
            arm.setRotation(ArmRotation.MidDeliver);
            arm.setGlobalWristRotation(true);
            arm.update();
            return false;
        };
    }

    public Action prepareForBackstageDelivery() {
        return packet -> {
            arm.setRotation(ArmRotation.BackDown);
            arm.setWristRotation(WristRotation.PickupBack);
            return false;
        };
    }

    public Action closeClaw() {
        return packet -> {
            claw.close();
            return false;
        };
    }
}
