package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions.Hammer;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions.Shoot;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions.Take;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.Actions.setVel;

public class IntakeSubsystems {
    private final IntakeIO io;
    private final Telemetry telemetry;

    private final HardwareMap hardwareMap;

    public IntakeSubsystems(HardwareMap hardwareMap, Telemetry telemetry) {
        io = new IntakeIO(hardwareMap);

        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;
    }

    public Action take() {
        return new Take(io, hardwareMap);
    }

    public Action shoot() {
        return new Shoot(io, hardwareMap);
    }

    public Action stop() {
        return new setVel(io, 0);
    }

    public Action hammer() {
        return new Hammer(io, hardwareMap);
    }

    // TODO: agregar acción que detenga el intake al entrar al estado travel
    public Action travel() {
        return null;
    }
    public void periodic() {


        telemetry.addData("Intake vel", io.getVelIntake());
        telemetry.addData("Index vel", io.getVelIndex());

        telemetry.addData("BlockerL pos", io.isBallDetected());

    }


}
