package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double r = 11;
        double w = 12;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(new Pose2d(18, -63.5, Math.toRadians(90)))
                                drive.trajectorySequenceBuilder(new Pose2d(0, -w, Math.toRadians(0)))
                                        //turn 1
                                        .splineTo(new Vector2d(w - r, -w), Math.toRadians(0))
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
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(50)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}