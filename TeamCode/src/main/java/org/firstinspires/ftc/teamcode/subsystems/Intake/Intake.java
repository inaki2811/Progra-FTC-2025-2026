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

    // shootea cuando ya esta listo el shooter
    public Action shootWhenReady(BooleanSupplier shooterReady) {
        return packet -> {
            if (shooterReady.getAsBoolean() ) {
                intakeWithIndex();
            } else {
                stopIntake();
            }
            return false;
        };
    }

}
