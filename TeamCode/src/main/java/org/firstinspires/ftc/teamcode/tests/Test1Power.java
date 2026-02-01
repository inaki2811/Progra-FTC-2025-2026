package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Test1Power", group="test")
@Config
public class Test1Power extends OpMode {
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

            launcherTop.setPower(1);
            launcherBottom.setPower(1);



        dashboardTelemetry.addData("Velocity in tps launcher top", launcherTop.getVelocity());
        dashboardTelemetry.addData("Velocity in tps launcher bottom", launcherBottom.getVelocity());
        dashboardTelemetry.addData("power", launcherBottom.getPower());
        dashboardTelemetry.addData("power", launcherTop.getPower());

        dashboardTelemetry.update();
    }
}