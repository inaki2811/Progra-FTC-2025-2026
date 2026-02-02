package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shoot;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import java.util.function.Supplier;

@Autonomous(name="AutonomusBlue", group="Testing")
public class AutonomousBlue extends LinearOpMode {

    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    public static Supplier<Pose2d> lastPose = () -> new Pose2d(-70, -59,  -Math.PI / 2);

    private VisionIO vision;
    private Intake intake;
    private Shoot shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa el drive y la posición inicial
        Pose2d startPose = new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        lastPose = drive.localizer::getPose;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        shooter = new Shoot(hardwareMap);
        intake = new Intake(hardwareMap);

        vision = new VisionIO(hardwareMap, shooter.getIO(), telemetry);

        Action prepareForShoot = shooter.prepareForShoot(() -> distanceWithTargetX(drive), () -> distanceWithTargetY(-1, drive), drive.localizer.getPose().heading::toDouble, () -> null, -0.9, telemetry);

        Action take = intake.take();
        Action shoot = intake.shoot();
        Action stop = intake.stopIntake();

        // Construye la acción (trajectory) del robot
        Action traj = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-30, -28), Math.atan2(distanceWithTargetYManual(-1, -28), distanceWithTargetXManual(-30)))

                .stopAndAdd(new SequentialAction(
                        stop,
                        prepareForShoot

                ))

                .stopAndAdd(new ParallelAction(
                        intake.stopIntake(),
                        shooter.stop()
                ))
                // PPG
                .strafeToLinearHeading(new Vector2d(PPG_POS, -28 + 3), -Math.PI / 2)
                .stopAndAdd(new ParallelAction(
                        intake.take(),
                        shooter.intake()
                ))
                .strafeToLinearHeading(new Vector2d(PPG_POS, -28 + 3 - 18), -Math.PI / 2)
                .stopAndAdd(stop)
                .strafeToLinearHeading(new Vector2d(-36.3, -31.5), Math.atan2(distanceWithTargetYManual(-1, -31.5), distanceWithTargetXManual(-36.3)))

                .stopAndAdd(new SequentialAction(
                        stop,
                        prepareForShoot

                ))

                // PGP
                .strafeToLinearHeading(new Vector2d(PGP_POS, -28 + 3), -Math.PI / 2)
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToLinearHeading(new Vector2d(PGP_POS, -28 + 3 - 18), -Math.PI / 2)
                .stopAndAdd(stop)
                .strafeToLinearHeading(new Vector2d(-7.9, -19.9), Math.atan2(distanceWithTargetYManual(-1, -19.9), distanceWithTargetXManual(-7.9)))

                .stopAndAdd(new SequentialAction(
                        stop,
                        prepareForShoot

                ))

                // GPP
                .strafeToLinearHeading(new Vector2d(GPP_POS, -28 + 3), -Math.PI / 2)
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToLinearHeading(new Vector2d(GPP_POS, -28 + 3 - 18), -Math.PI / 2)
                .stopAndAdd(stop)
                .strafeToLinearHeading(new Vector2d(54.3, -12.9), Math.atan2(distanceWithTargetYManual(-1, -12.9), distanceWithTargetXManual(54.3)))

                .stopAndAdd(new SequentialAction(
                        stop,
                        prepareForShoot

                ))

                .stopAndAdd(new ParallelAction(
                        intake.stopIntake(),
                        shooter.stop()
                ))

                .build();

        // Espera al inicio del opmode
        waitForStart();
        if (isStopRequested()) return;

        // Ejecuta la acción y actualiza la pose en tiempo real
        Actions.runBlocking(traj);
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
