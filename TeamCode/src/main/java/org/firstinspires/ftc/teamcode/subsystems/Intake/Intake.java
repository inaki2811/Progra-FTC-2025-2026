package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Intake {
    private final IntakeIO io;
    private final ElapsedTime timer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
        io = new IntakeIO(hardwareMap);
    }

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

    public Action take() {
        return packet -> {
            if (!isBallDetected()){
                intakeWithIndex();
            } else if (isBallDetected()){
                intakeWithoutIndex();
            }
            return false; // Corre continuamente
        };
    }

    public Action reverseAction() {
        return packet -> {
            reverse();
            return false; // Corre continuamente
        };
    }

    public Action stopIntake() {
        return packet -> {
            stop();
            return true; // ✅ Termina inmediatamente (esto está bien)
        };
    }

    public Action shoot() {
        return packet -> {
            intakeWithIndex();
            return true;
        };
    }

    public Action shootWhenReady(
            BooleanSupplier shooterReady,
            DoubleSupplier currentVel,
            double threshold,
            double SHOOT_READY_VEL,
            double MIN_FEED_TIME,
            double MAX_FEED_TIME) {

        // Timer que persiste entre llamadas
        ElapsedTime feedTimer = new ElapsedTime();

        return new Action() {
            private boolean feeding = false;
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    feedTimer.reset();
                    initialized = true;
                }

                double vel = currentVel.getAsDouble();
                boolean ready = shooterReady.getAsBoolean();
                double elapsed = feedTimer.seconds();

                packet.put("🎯 Shoot Ready", ready);
                packet.put("📊 Shooter Vel", vel);
                packet.put("⏱️ Feed Time", elapsed);

                // Si el shooter está listo y tiene velocidad suficiente
                if (ready && vel >= threshold) {
                    if (!feeding) {
                        feeding = true;
                        feedTimer.reset();
                    }
                    intakeWithIndex();

                    // Termina después del tiempo mínimo de alimentación
                    if (elapsed >= MIN_FEED_TIME) {
                        stop();
                        return true; // ✅ Terminó de disparar
                    }
                } else {
                    stop();
                    feeding = false;
                }

                // Timeout de seguridad
                if (elapsed >= MAX_FEED_TIME) {
                    stop();
                    return true; // ✅ Timeout alcanzado
                }

                return false; // ❌ Sigue esperando
            }
        };
    }
}