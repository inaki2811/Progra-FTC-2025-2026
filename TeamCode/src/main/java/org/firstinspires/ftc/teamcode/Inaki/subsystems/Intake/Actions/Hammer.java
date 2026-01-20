package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;

public class Hammer implements Action {
    private final IntakeIO io;
    private int lastUsedStage = 0;
    private Servo hammerShooter;
    private ElapsedTime elapsedTime;
    private boolean initialized = false;

    public Hammer(IntakeIO io, HardwareMap hardwareMap) {
        this.io = io;
        hammerShooter = hardwareMap.get(Servo.class, "hammer");hammerShooter.setPosition(1);

    }


    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            elapsedTime = new ElapsedTime();
            initialized = true;
        }
        hammerShooter.setPosition(0);

        return !isFinished();
    }

    public void onEnd() {

    }

    public boolean isFinished() {
        return elapsedTime.milliseconds() > 1000;
    }
}
