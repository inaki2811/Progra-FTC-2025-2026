package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@Autonomous(name="AutonomusRed", group="Testing")
public class AutonomousRed extends LinearOpMode {

    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    public static Supplier<Pose2d> lastPose = () -> new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);

    private AprilTagDetection tagDetection;
    private VisionIO vision;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa el drive y la posición inicial
        Pose2d startPose = new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        ShooterSubsystems shooter = new ShooterSubsystems(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        vision = new VisionIO(hardwareMap, shooter.getIO(), telemetry);

        lastPose = drive.localizer::getPose;

        Action prepareForShoot = shooter.prepareForShoot(
                () -> -64 - drive.localizer.getPose().position.x,
                () -> (59 - drive.localizer.getPose().position.y),
                () -> drive.localizer.getPose().heading.toDouble(),
                ()-> vision.getTagBySpecificId(24),
                2.5,
                telemetry
        );
        Action take = intake.take();
        Action shoot = intake.shoot();
        Action stop = intake.stopIntake();

        // Construye la acción (trajectory) del robot
        Action traj = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-30, 28))

                .stopAndAdd(new SequentialAction(
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 2.5,telemetry),
                        intake.shoot(),
                        intake.stopIntake(),
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 0.25, telemetry)
                ))

                // PPG
                .strafeToConstantHeading(new Vector2d(PPG_POS, 28 - 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(PPG_POS, 28 - 3 + 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(-36.3, 31.5))
                .stopAndAdd(new SequentialAction(
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 2.5,telemetry),
                        intake.shoot(),
                        intake.stopIntake(),
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, -0.9, telemetry)
                ))

                // PGP
                .strafeToConstantHeading(new Vector2d(PGP_POS, 28 - 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(PGP_POS, 28 - 3 + 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(-7.9, 19.9))
                .stopAndAdd(new SequentialAction(
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 2.5,telemetry),
                        intake.shoot(),
                        intake.stopIntake(),
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble,() ->  tagDetection, 0.25, telemetry)
                ))

                // GPP
                .strafeToConstantHeading(new Vector2d(GPP_POS, 28 - 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(GPP_POS, 28 - 3 + 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(54.3, 12.9))
                .stopAndAdd(new SequentialAction(
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 2.5,telemetry),
                        intake.shoot(),
                        intake.stopIntake(),
                        shooter.prepareForShoot(() -> -64 - (drive.localizer.getPose().position.x), () -> ((59 -  (drive.localizer.getPose().position.y))), drive.localizer.getPose().heading::toDouble, () -> tagDetection, 0.25, telemetry)
                ))

                .build();

        // Espera al inicio del opmode
        waitForStart();
        if (isStopRequested()) return;

        // Ejecuta la acción y actualiza la pose en tiempo real
        while (opModeIsActive()) {
            Actions.runBlocking(traj);

            vision.update();
            vision.displayDetectionTelemetry(vision.getTagBySpecificId(24));

            telemetry.update();
        }
    }
}
