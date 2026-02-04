package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shoot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

public class SubsystemManager {
    private final Shoot shooter;
    private final Intake intake;
    private final Queue<RobotState> stateQueue = new LinkedList<>();

    // Actions activas para el estado actual (no recrear cada tick)
    private Action runningAction = null;
    private RobotState cachedState = null;
    private final Telemetry telemetry;
    private final int allianceMult = -1;

    // Opcional: Referencia a un gestor de visión si lo tienes
    // private final VisionManager vision;

    public SubsystemManager(HardwareMap hardwareMap,Telemetry telemetry,MecanumDrive drive,Supplier<AprilTagDetection> tagDetection) {
        this.telemetry = telemetry;
        ShooterIO shooterIO = new ShooterIO(hardwareMap);
        intake = new Intake(hardwareMap);

        Supplier<Double> distanceX = () -> ((-62 + 14.57) - drive.localizer.getPose().position.x) * 0.0254;
        Supplier<Double> distanceY = () -> ((60 + 15.35) * allianceMult - drive.localizer.getPose().position.y) * 0.0254;
        Supplier<Double> botYaw = () -> drive.localizer.getPose().heading.toDouble();

        shooter = new Shoot(shooterIO,distanceX,distanceY,botYaw,tagDetection,-0.9, telemetry);
    }

    public void scheduleState(RobotState state) {
        if (state != null) stateQueue.add(state);
    }

    public void setState(RobotState state) {
        stateQueue.clear();
        if (state != null) stateQueue.add(state);
    }

    /**
     * periodic debe recibir el TelemetryPacket por tick (dashboard) y pasar
     * exactame|nte ese packet a las Actions que ejecutamos.
     */
    public void periodic(TelemetryPacket telemetryPacket) {
        if (stateQueue.isEmpty()) {
            telemetry.addData("State", "IDLE");
            return;
        }

        RobotState current = stateQueue.peek();

        // Si entramos en un nuevo estado, inicializamos las Actions asociadas UNA VEZ
        if (cachedState != current) {
            // limpiar cualquier action previa (por seguridad)
            runningAction = null;
            cachedState = current;

            switch (current) {

                case INTAKE:
                    runningAction = new ParallelAction(
                            intake.take()
                    );
                    break;

                case SHOOT:
                    runningAction = new SequentialAction(
                            intake.stopIntake(),
                            new ParallelAction(
                                    shooter.prepareVelocity(),
                                    shooter.trackingYaw(),
                                    shooter.trackingHood()
                            ),
                            intake.shootWhenReady(
                                    shooter::atVelocity,
                                    shooter::getVel,
                                    530,
                                    550,
                                    0.3,
                                    3.0
                            )
                            );
                    break;

                case STOP:
                    runningAction = new ParallelAction(
                            intake.stopIntake(),
                            shooter.stop()
                    );
                    break;

                default:
                    runningAction = packet1 -> true;
            }
        }

        if (runningAction != null) {
            boolean finished = runningAction.run(telemetryPacket);
            if (finished) {
                stateQueue.poll();
                cachedState = null;
                runningAction = null;
            }
        }

        telemetry.addData("State", current.name());
    }

}

