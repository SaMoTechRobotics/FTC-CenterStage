package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-30, 63.5, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(270)))
                                        .splineToSplineHeading(new Pose2d(-30, 40, Math.toRadians(270 + 45)), Math.toRadians(270 + 45))
                                        .waitSeconds(1)
//                                        .splineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(270 + 45)), Math.toRadians(270 + 45))
//                                        .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(270 + 45)))
                                        .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(270 + 45)))
                                        .splineToSplineHeading(new Pose2d(-24, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(0, 12, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(180)))
                                        .splineToLinearHeading(new Pose2d(39, 36, Math.toRadians(180)), Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)), Math.toRadians(180))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}