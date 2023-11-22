package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public final class DriftAccuracyTest extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(DISTANCE, 0), 0)
                            .strafeToLinearHeading(new Vector2d(0, 0), 0)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(0, 0), 0)
                            .build());
        }
    }
}
