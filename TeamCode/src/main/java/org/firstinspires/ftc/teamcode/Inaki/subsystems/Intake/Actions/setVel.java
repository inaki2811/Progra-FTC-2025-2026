package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;


public class setVel implements Action {
    private final IntakeIO io;

    private final double velIntake;
    private final double velIndex;

    public setVel(IntakeIO io, double vel) {
        this.io = io;
        this.velIntake = vel;
        this.velIndex = vel;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setVelIntake(velIntake);
        io.setVelIndex(velIndex);
        return !isFinished();
    }

    public boolean isFinished() {
        return true;
    }
}
