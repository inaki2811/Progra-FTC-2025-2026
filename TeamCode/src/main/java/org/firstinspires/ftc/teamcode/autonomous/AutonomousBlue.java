package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import java.util.function.Supplier;

@Autonomous(name="AutonomusBlue", group="Testing")
public class AutonomousBlue extends LinearOpMode {

    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;
    public static Supplier<Pose2d> lastPose = () -> new Pose2d(-70, -59,  -Math.PI / 2);
    private VisionIO vision;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa el drive y la posición inicial
        Pose2d startPose = new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        lastPose = drive.localizer::getPose;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        intake = new Intake(hardwareMap);



        Action take = intake.take();
        Action shoot = intake.shoot();
        Action stop = intake.stopIntake();

        // Construye la acción (trajectory) del robot
        Action traj1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-13, -22), -Math.PI / 2)
                .build();

        Action traj2 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(-11.6, -50) )
                .strafeToLinearHeading(new Vector2d(-20,-22), Math.atan2(distanceWithTargetYManual(1,-22),distanceWithTargetXManual(20)))
                .build();

        Action traj3 = drive.actionBuilder(startPose)
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

    private double distanceWithTargetX(MecanumDrive drive) {
        double distance = ((-64 +  14.57) - drive.localizer.getPose().position.x) * 0.0254;
        telemetry.addData("distance with target X", distance);
        return distance;
    }

    private double distanceWithTargetY(double allianceMult, MecanumDrive drive) {
        double distance = ((59 + 15.35)  * allianceMult -  (drive.localizer.getPose().position.y)) * 0.0254;
        telemetry.addData("distance with target Y", distance);
        return distance;
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
