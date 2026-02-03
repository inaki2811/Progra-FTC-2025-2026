package org.firstinspires.ftc.teamcode.autonomous;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class TrajectoryTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action traj1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-13, -22), -Math.PI / 2)
                .build();

        Action traj2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-11.6, -50) )
                .strafeToLinearHeading(new Vector2d(-20,-22), Math.atan2(distanceWithTargetYManual(1,-22),distanceWithTargetXManual(20)))
                .build();

        Action traj3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(12.2,-22), Math.PI / 2)
                .strafeToConstantHeading(new Vector2d(12.2, -50))
                .strafeToLinearHeading(new Vector2d(-20,-22), Math.atan2(distanceWithTargetYManual(1,-22),distanceWithTargetXManual(20)))
                .build();


        // Espera al inicio del opmode
        waitForStart();
        if (isStopRequested()) return;

        // Ejecuta la acción y actualiza la pose en tiempo real
        Actions.runBlocking(new SequentialAction(traj1, traj2, traj3));
    }




    private double distanceWithTargetXManual(double x) {

        double distance = ((-64 +  14.57) - x) * 0.0254;
        telemetry.addData("distance with target X", distance);
        return distance;
    }

    private double distanceWithTargetYManual(double allianceMult, double y) {
        double distance = ((59 + 15.35)  * allianceMult - (y)) * 0.0254;
        telemetry.addData("distance with target Y", distance);
        return distance;
    }
}
