package org.firstinspires.ftc.teamcode.Inaki.core;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeSubsystems;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.ShooterSubsystems;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

public class SubsystemManager {
    private final ShooterSubsystems shooter;
    private final IntakeSubsystems intake;
    private final Queue<RobotState> stateQueue;

    // Actions activas para el estado actual (no recrear cada tick)
    private Action runningAction = null;
    private RobotState cachedState = null;

    private final Telemetry telemetry;
    private final int allianceMult = -1;

    // Opcional: Referencia a un gestor de visión si lo tienes
    // private final VisionManager vision;

    public SubsystemManager(HardwareMap hardwareMap, Telemetry telemetry) {
        shooter = new ShooterSubsystems(hardwareMap);
        intake = new IntakeSubsystems(hardwareMap, telemetry);
        stateQueue = new LinkedList<>();

        this.telemetry = telemetry;
    }

    public void scheduleState(RobotState state) {
        if (state == null) return;
        stateQueue.add(state);
    }

    public void setState(RobotState state) {
        if (state == null) return;
        stateQueue.clear();
        stateQueue.add(state);
    }

    /**
     * periodic debe recibir el TelemetryPacket por tick (dashboard) y pasar
     * exactame|nte ese packet a las Actions que ejecutamos.
     */
    public void periodic(MecanumDrive drive, Supplier<AprilTagDetection> tagDetection, TelemetryPacket telemetryPacket, int alianceMult) {
        if (stateQueue.isEmpty()) {
            scheduleState(RobotState.TRAVEL);
        }

        RobotState current = stateQueue.peek();

        distanceWithTargetX(drive, telemetry);
        distanceWithTargetY(drive, telemetry, alianceMult);

        // Si entramos en un nuevo estado, inicializamos las Actions asociadas UNA VEZ
        if (cachedState != current) {
            // limpiar cualquier action previa (por seguridad)
            runningAction = null;
            cachedState = current;

            switch (current) {
                case TRAVEL:
                    runningAction = null;
                    break;

                case INTAKE:
                    runningAction = new ParallelAction(
                            intake.take(),
                            shooter.intake()
                    );
                    break;

                case SHOOT:
                    runningAction = new SequentialAction(
                            intake.stop(),
                            shooter.prepareForShoot(() -> distanceWithTargetX(drive, telemetry), () -> distanceWithTargetY(drive, telemetry, alianceMult), drive.localizer.getPose().heading::toDouble, tagDetection, -0.9, telemetry),
                            intake.hammer()
                    );
                    break;
                case STOP:
                    runningAction = new ParallelAction(
                            intake.stop(),
                            shooter.stop()
                    );
            }
        }

        if (runningAction != null) {
           runningAction.run(telemetryPacket);
        }

        intake.periodic();
        shooter.periodic(telemetry);

        telemetry.addData("State", current.toString());
    }
    private double distanceWithTargetX(MecanumDrive drive, Telemetry telemetry) {
        double distance = ((-62 +  14.57) - (drive.localizer.getPose().position.x)) * 0.0254;
        telemetry.addData("distance with target X", distance);
        return distance;
    }

    private double distanceWithTargetY(MecanumDrive drive, Telemetry telemetry, double allianceMult) {
        double distance = ((60 + 15.35)  * allianceMult -  (drive.localizer.getPose().position.y)) * 0.0254;
        telemetry.addData("distance with target Y", distance);
        return distance;
    }
}

