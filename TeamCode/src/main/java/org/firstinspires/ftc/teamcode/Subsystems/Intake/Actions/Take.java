package org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;

public class Take implements Action {
    private final IntakeIO io;
    private int lastUsedStage = 0;
    private Servo hammerShooter;

    public Take(IntakeIO io, HardwareMap hardwareMap) {
        this.io = io;
        hammerShooter = hardwareMap.get(Servo.class, "hammerS");hammerShooter.setPosition(1);

    }


    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setPwr(-1);
        hammerShooter.setPosition(1);
        //io.setBlockState(true);

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
