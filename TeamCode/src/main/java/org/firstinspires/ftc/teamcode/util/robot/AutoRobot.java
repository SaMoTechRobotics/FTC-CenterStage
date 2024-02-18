package org.firstinspires.ftc.teamcode.util.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.robot.arm.Arm;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
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

    public Action openNextClaw() {
        return packet -> {
            claw.openNext();
            return false;
        };
    }

    public Action raiseArmForStack() {
        return packet -> {
            arm.setRotation(ArmRotation.Stack5);
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
