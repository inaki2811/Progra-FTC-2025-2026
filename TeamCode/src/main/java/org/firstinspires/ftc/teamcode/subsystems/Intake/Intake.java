package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.BooleanSupplier;

public class Intake {
    private final IntakeIO io;
    private final ElapsedTime timer = new ElapsedTime();
    public Intake(HardwareMap hardwareMap) {
        io = new IntakeIO(hardwareMap);
    }

    // métodos sencillos, come y traga el intake

    public void intake() {
        io.setPwr(1);
    }

    public void reverse() {
        io.setPwr(-1);
    }

    public void stop() {
        io.setPwr(0);
    }

    public boolean hasBallReady() {
        return io.onStage3();
    }

    // acciones para autónomos y road runner, para llamarlas

    // intake come
    public Action take() {
        return packet -> {
            intake();
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

                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                if (timer.milliseconds() < 60) {
                    io.setPwr(1.0);
                }
                else if (timer.milliseconds() < 300) {
                    io.setPwr(-0.6);
                }
                else {
                    io.setPwr(0.0);
                    timer.reset();
                }
                return true;
            }
        };
    }

    // shootea cuando ya esta listo el shooter
    public Action shootWhenReady(BooleanSupplier shooterReady) {
        return packet -> {
            if (shooterReady.getAsBoolean() && hasBallReady()) {
                io.setPwr(1.0);
            } else {
                io.setPwr(0.0);
            }
            return false;
        };
    }

}
