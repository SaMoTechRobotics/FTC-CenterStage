package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int side = 1;
        int color = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, color * 63.5, -color * Math.toRadians(180)))
//                                        .splineToLinearHeading(new Pose2d(-40, color * 12, -color * Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(0, color * 12, -color * Math.toRadians(270)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(36, color * -20, color * Math.toRadians(120)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(36, color * 36, -color * Math.toRadians(270)), Math.toRadians(0))
                                        .waitSeconds(2)
//                                        .strafeLeft(24)
//                                        .back(24)
                                        .splineToSplineHeading(new Pose2d(34, 12, Math.toRadians(270)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63, 12, Math.toRadians(270)), Math.toRadians(0))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}