package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="JoystickMotor", group="test")
@Config
public class JoystickMotor extends OpMode {
    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;



    Telemetry dashboardTelemetry;

    @Override
    public void init() {
        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {

            launcherTop.setPower(-gamepad1.left_stick_y);
            launcherBottom.setPower(-gamepad1.left_stick_y);



        dashboardTelemetry.addData("Velocity in tps launcher top", launcherTop.getVelocity());
        dashboardTelemetry.addData("Velocity in tps launcher bottom", launcherBottom.getVelocity());
        dashboardTelemetry.addData("power", launcherBottom.getPower());
        dashboardTelemetry.addData("power", launcherTop.getPower());

        dashboardTelemetry.update();
    }
}