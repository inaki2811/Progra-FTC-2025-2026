package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Yaw;

@TeleOp(name="TestYawCalibration", group="Test")
public class TestYawCalibration extends OpMode {
    private Yaw yaw;

    @Override
    public void init() {
        yaw = new Yaw();
        yaw.init(hardwareMap);
    }

    @Override
    public void loop() {
        double input = gamepad1.right_stick_x;
        double targetAngle = input * 60; // ±30 grados

        yaw.setTargetAngleDeg(targetAngle);

        telemetry.addData("Joystick Input", input);
        telemetry.addData("Turret Angle (deg)", yaw.getTurretAngleDeg());
        telemetry.addData("Camera Yaw (rad)", yaw.getCameraYawRad());
        telemetry.update();
    }

    @Override
    public void stop() {
        yaw.stop();
    }
}