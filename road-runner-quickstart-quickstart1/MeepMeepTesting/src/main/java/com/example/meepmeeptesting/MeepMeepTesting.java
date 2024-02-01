package com.example.meepmeeptesting;

import static jdk.jfr.internal.consumer.EventLog.stop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44.9, 30, 44.9, 1.047, 11.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.50, -63.00, Math.toRadians(90.00)))
                                .UNSTABLE_addTemporalMarkerOffset(1.70,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(2.20,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.70,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(4.80,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(6.00,() -> {})
                                .splineTo(new Vector2d(6.50, -37.50), Math.toRadians(135.00))
                                .lineToSplineHeading(new Pose2d(51.50, -37.50, Math.toRadians(180.00)))
                                .waitSeconds(2.2)
                                .lineTo(new Vector2d(51.50, -63.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}