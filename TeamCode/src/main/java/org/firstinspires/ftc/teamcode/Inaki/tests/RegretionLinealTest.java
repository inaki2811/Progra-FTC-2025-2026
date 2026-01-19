package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Hood;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.ShooterIO;

@TeleOp(name="RegretionLinealTest", group="Test")
public class RegretionLinealTest extends OpMode {
    private ShooterIO shooterIO;
    private Hood hood;

    @Override
    public void init() {
        shooterIO = new ShooterIO();
        shooterIO.init(hardwareMap);

        hood = new Hood();
        hood.init(hardwareMap);
    }

    @Override
    public void loop() {
        double targetVelocity = 3000;
        double hoodAngle = 0.25;

        shooterIO.setTargetVelocity(targetVelocity);
        hood.setTarget(hoodAngle);

        telemetry.addData("Target RPM", targetVelocity);
        telemetry.addData("Actual RPM", shooterIO.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterIO.stop();
        hood.stop();
    }
}