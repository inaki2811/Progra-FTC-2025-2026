package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;

public class Take implements Action {
    private final IntakeIO io;
    private Servo hammerShooter;

    public Take(IntakeIO io, HardwareMap hardwareMap) {
        this.io = io;
        hammerShooter = hardwareMap.get(Servo.class, "hammer");
        hammerShooter.setPosition(0);

    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        hammerShooter.setPosition(0);

        if (!io.isBallDetected()){
            io.setPwrIntake(0.01);
            io.setPwrIndex(0.1);
        }else{
            io.setPwrIntake(0.01);
            io.setPwrIndex(0);

        }

        if (isFinished()) {
            onEnd();
        }

        return !isFinished();
    }

    public void onEnd() {

    }

    public boolean isFinished() {
        return false;
    }
}
