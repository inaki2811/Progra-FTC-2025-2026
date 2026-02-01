package org.firstinspires.ftc.teamcode.Inaki.tests;

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

@TeleOp(name="TuneShooterVelocity", group="test")
@Config
public class TuneShooterVelocity extends OpMode {
    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;

    public static double vel = 720;
    public static double openLoopPower = 0.75;
    private Servo hammerShooter;

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            0.015, 0.9, 0.0003, 0
    );

    public static double kA = 0.0007;

    public static final PIDFController shooterController = new PIDFController(shooterCoeffs);

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

        shooterController.setCoefficients(shooterCoeffs);

        shooterController.setTolerance(20);
        shooterController.setSetPoint(vel);

        double currentVelocity = launcherTop.getVelocity();

        double power = (kA * vel) + shooterController.calculate(currentVelocity);

        power = Math.max(-1, Math.min(power, 1));


        // A → velocity mode

            launcherTop.setPower(power);
            launcherBottom.setPower(power);


        // B → open loop
        if (gamepad1.b) {
            launcherTop.setPower(openLoopPower);
            launcherBottom.setPower(openLoopPower);
        }


        dashboardTelemetry.addData("Velocity in tps launcher top", launcherTop.getVelocity());
        dashboardTelemetry.addData("Velocity in tps launcher bottom", launcherBottom.getVelocity());
        dashboardTelemetry.addData("power", power);
        dashboardTelemetry.addData("shooter sepoint", shooterController.getSetPoint());
        dashboardTelemetry.addData("error", shooterController.getPositionError());

        dashboardTelemetry.update();
    }
}