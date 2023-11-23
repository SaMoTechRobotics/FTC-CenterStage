package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;

@Config
public final class SplineTest extends LinearOpMode {
    public static double r = 8;
    public static double w = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(0, -w), 0)
                                .build());
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .lineToX(w - r)
                                //turn 1
//                                .splineTo(new Vector2d(w - r, -w), Math.toRadians(0))
                                .splineTo(new Vector2d(w, -(w - r)), Math.toRadians(90))
                                //turn 2
                                .splineTo(new Vector2d(w, w - r), Math.toRadians(90))
                                .splineTo(new Vector2d(w - r, w), Math.toRadians(180))
                                //turn 3
                                .splineTo(new Vector2d(-(w - r), w), Math.toRadians(180))
                                .splineTo(new Vector2d(-w, w - r), Math.toRadians(270))
                                //turn 4
                                .splineTo(new Vector2d(-w, -(w - r)), Math.toRadians(270))
                                .splineTo(new Vector2d(-(w - r), -w), Math.toRadians(360))
                                .splineTo(new Vector2d(0, -w), Math.toRadians(360))
//                            .splineTo(new Vector2d(28, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 40), Math.PI)
//                            .splineTo(new Vector2d(28, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 0), 0)
                                .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        } else {
            throw new AssertionError();
        }
    }
}
