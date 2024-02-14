package org.firstinspires.ftc.teamcode.util.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.robot.arm.Arm;
import org.firstinspires.ftc.teamcode.util.robot.claw.Claw;
import org.firstinspires.ftc.teamcode.util.vision.Vision;

public class AutoRobot {
    private final Telemetry telemetry;

    public Vision vision;

    public MecanumDrive drive;

    public Arm arm;
    public Claw claw;

    public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose) {
        vision = new Vision(hardwareMap);

        drive = new MecanumDrive(hardwareMap, pose);

        arm = new Arm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }

    public void setPose(Pose2d pose) {
        drive.pose = pose;
    }
}
