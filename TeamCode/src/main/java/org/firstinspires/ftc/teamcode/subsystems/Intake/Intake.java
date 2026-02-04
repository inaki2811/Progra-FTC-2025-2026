package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Intake {
    private final IntakeIO io;
    private final ElapsedTime timer = new ElapsedTime();
    public Intake(HardwareMap hardwareMap) {
        io = new IntakeIO(hardwareMap);
    }

    // métodos sencillos, come y traga el intake

    public void intakeWithIndex() {
        io.setPwrIntake(0.6);
        io.setPwrIndex(0.8);
    }

    public void intakeWithoutIndex() {
        io.setPwrIntake(0.4);
        io.setPwrIndex(0);
    }

    public void reverse() {
        io.setPwrIntake(-1);
        io.setPwrIndex(-1);
    }

    public void stop() {
        io.setPwrIntake(0);
        io.setPwrIndex(0);
    }

    public boolean isBallDetected() {
        return io.getDistanceSensor() < 7;
    }

    // acciones para autónomos y road runner, para llamarlas

    // intake come
    public Action take() {
        return packet -> {

            if (!isBallDetected()){
                intakeWithIndex();
            }else{
                intakeWithoutIndex();

            }

            return false;
        };
    }

    // intake escupe
    public Action reverseAction() {
        return packet -> {
            reverse();
            return false;
        };
    }

    // intake para
    public Action stopIntake() {
        return packet -> {
            stop();
            return true;
        };
    }

    // shootea n cantidad de pelotas
    public Action shoot() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {

                    intakeWithIndex();

                return true;
            }
        };
    }



    public Action shootWhenReady(BooleanSupplier shooterReady,
                                 DoubleSupplier currentVel,
                                 double threshold,
                                 double SHOOT_READY_VEL,
                                 double MIN_FEED_TIME,
                                 double MAX_FEED_TIME) {

        // Creamos un timer específico para esta acción
        ElapsedTime feedTimer = new ElapsedTime();
        feedTimer.reset();

        return packet -> {
            // Solo alimentar si shooter está "ready" y velocidad en rango
            if (shooterReady.getAsBoolean()
                    && currentVel.getAsDouble() >= threshold
                    && currentVel.getAsDouble() <= SHOOT_READY_VEL) {
                intakeWithIndex();
            } else {
                stopIntake();
            }

            // Condición de disparo completo
            if (currentVel.getAsDouble() < threshold
                    && feedTimer.seconds() >= MIN_FEED_TIME) {
                stopIntake();
                // aquí podrías apagar shooter o cambiar estado
                return true; // acción terminada
            }

            // Timeout de seguridad
            if (feedTimer.seconds() >= MAX_FEED_TIME) {
                stopIntake();
                // aquí también podrías apagar shooter o cambiar estado
                return true; // acción terminada
            }

            return false; // acción sigue activa
        };
    }

}
