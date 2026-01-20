package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;

public class Shoot implements Action {
    private final IntakeIO io;
    private boolean initialized = false;
    private int iteration = 0;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private Servo hammerShooter;

    public Shoot(IntakeIO io, HardwareMap hardwareMap) {
        this.io = io;
        hammerShooter = hardwareMap.get(Servo.class, "hammerS");
        hammerShooter.setPosition(1);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            if (elapsedTime.milliseconds() > 2000) {
                initialized = true;
                elapsedTime.reset();
            }
        } else {
            if (elapsedTime.milliseconds() < 50) {
                io.setPwrIndex(1);
                io.setPwrIntake(1);
            } else if (elapsedTime.milliseconds() < 750) {
                io.setPwrIndex(1);
                io.setPwrIntake(1);
                hammerShooter.setPosition(0);

            } else {
                io.setPwrIndex(0);
                io.setPwrIndex(0);

                iteration++;
                elapsedTime.reset();
            }
        }

        if (isFinished()) {
            onEnd();
        }

        return !isFinished();
    }

    public void onEnd() {
        io.setPwrIndex(0);
        io.setPwrIndex(0);
        hammerShooter.setPosition(1);
    }

    public boolean isFinished() {
        return iteration > 4;
    }
}

