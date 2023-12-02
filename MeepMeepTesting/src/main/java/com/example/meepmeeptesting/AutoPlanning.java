package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-24 - 6.5 - 1, 63.5, Math.toRadians(270)))
//                                drive.trajectorySequenceBuilder(new Pose2d(0 + 6.5 - 1, 63.5, Math.toRadians(90)))
//                                        .turn(Math.toRadians(180))
                                        //right
//                                        .lineToLinearHeading(new Pose2d(-46, 40, Math.toRadians(270)))
//                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(-30, 60, Math.toRadians(180)), Math.toRadians(0))
//                                        .splineToSplineHeading(new Pose2d(-36, 40, Math.toRadians(270)), Math.toRadians(0))
//                                        .turn(Math.toRadians(-30))
//                                        .forward(10)
//                                        .back(10)
//                                        .turn(Math.toRadians(30))
//                                        .back(36)
//                                        .turn(Math.toRadians(30))

                                        //center
//                                        .lineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(150)))
                                        //left
                                        .lineToLinearHeading(new Pose2d(-40, 48, Math.toRadians(-90)))
                                        .turn(Math.toRadians(-30))
                                        .forward(8)
                                        .back(8)
                                        .turn(Math.toRadians(30))
                                        .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(-90)))
//                                        .lineToLinearHeading(new Pose2d(-28, 36, Math.toRadians(-60)))
//                                        .lineToLinearHeading(new Pose2d(-32, 34, Math.toRadians(-90)))

//                                .turn(Math.toRadians(-30))
//                                .forward(6)
//                                .turn(Math.toRadians(30))
//                                .back(6)
//                                        .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(90)))
//                                        .forward(6)
//                                        .back(6)
//                                        .lineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(90)))

//                                        .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(120)))
//                                        .strafeTo(new Vector2d(-36, 12))
                                        .turn(Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(0, 12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(44, 36, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(62, 12, Math.toRadians(180)))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}