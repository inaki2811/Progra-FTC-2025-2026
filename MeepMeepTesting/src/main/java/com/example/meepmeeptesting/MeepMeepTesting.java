package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    private static final Pose2d INITIAL_POS_RED_1 = new Pose2d(-23.6 - 9, 24.6 + 9,  Math.PI / 2);
    private static final Pose2d INITIAL_POS_RED_2 = new Pose2d(70 - 8, 24.6 - 9,  Math.PI / 2);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16,17)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2))


                .strafeTo(new Vector2d(-13, -22))
                .waitSeconds(3)/*
                .strafeTo(new Vector2d(-11.6, -50) )
                .strafeTo(new Vector2d(-13,-22))
                .waitSeconds(3)*/
                .strafeTo(new Vector2d(12.2,-28))
                .strafeTo(new Vector2d(12.2, -50))
                .strafeTo(new Vector2d(-13,-22))
                .turn( -Math.PI / 4)
                        .strafeTo(new Vector2d(10,-57))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(-13,-22))
                        .waitSeconds(3)
                        .strafeTo(new Vector2d(10,-57))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(-13,-22))
                        .waitSeconds(3)
                        .strafeTo(new Vector2d(10,-57))
                        .waitSeconds(0.5)
                       .strafeTo(new Vector2d(-13,-22))
                        .waitSeconds(3)




                        .build());



        // Espera al inicio del opmode


        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\backgrounds\\field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}