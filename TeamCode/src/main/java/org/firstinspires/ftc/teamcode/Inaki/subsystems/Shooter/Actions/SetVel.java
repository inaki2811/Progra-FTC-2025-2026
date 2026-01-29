package org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.ShooterIO;

public class SetVel implements Action {
    private final ShooterIO io;

    private final double vel;

    public SetVel(ShooterIO io, double vel) {
        this.io = io;
        this.vel = vel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setPoint(vel);
        io.setVel();
        return !isFinished();
    }

    public boolean isFinished() {
        return true;
    }
}
