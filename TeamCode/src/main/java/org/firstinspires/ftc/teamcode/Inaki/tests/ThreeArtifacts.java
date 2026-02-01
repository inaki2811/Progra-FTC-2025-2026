package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@TeleOp(name = "ThreeArtifacts", group = "test")
@Config
public class ThreeArtifacts extends OpMode {

    private DcMotorEx Top, Down, Intake, Index;
    private MecanumDrive drive;
    private Servo hood;

    private double pitch;
    public static double distance = 0;
    public static double vel = 720;                 // setpoint (tps)
    public static double openLoopPower = 0.75;
    public static double offsetX = 0; //14.57
    public static double offsetY = 0; //15.35
    public static double intakePower = 0.4;
    public static double indexPower = -1;

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            0.09, 0.9, 0.000001, 0
    );
    public static double kA = 0.0007;               // feedforward gain (simple)
    public static double tolerance = 50;
    public static final PIDFController shooterController = new PIDFController(shooterCoeffs);

    // límites de potencia aplicable a los motores
    public static double minPower = -1.0;
    public static double maxPower = 1.0;

    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        Top = hardwareMap.get(DcMotorEx.class, "launcherTop");
        Down = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        Top.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);



        // Usamos control por potencia (PID externo) sobre RUN_WITHOUT_ENCODER
        Top.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Down.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(1);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        Intake.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);

        Index = hardwareMap.get(DcMotorEx.class, "index");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        // Inicializamos el controlador

    }

    @Override
    public void loop() {
        // Actualizamos setpoint por si se cambia desde dashboard
        Top.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterCoeffs);
        Down.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterCoeffs);

        // Movimiento del robot
        double driveY = -gamepad1.left_stick_x;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_y;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        double heading = -pose.heading.toDouble() - Math.toRadians(180);

        double rotatedX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
        double rotatedY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotatedX, rotatedY),
                turn
        ));

        if (gamepad1.dpad_down) {
            drive.localizer.setPose(new Pose2d(-70, -59, -Math.PI / 2));
        }

        hood.setPosition(pitch);
        distance = Math.hypot(
                distanceWithTargetX(drive, telemetry),
                distanceWithTargetY(drive, telemetry, 1)
        );



        // Control PID + feedforward
        // Nota: si la API de PIDFController que usas tiene otro método para calcular la sali

        // Feedforward simple proporcional a la aceleración requerida.


        Top.setVelocity(vel);
        Down.setVelocity(vel);
        // Mantener intake e index
        Intake.setPower(intakePower);
        Index.setPower(indexPower);

        // Cálculo de pitch en función de la distancia (misma regresión que tenías)
        if (distance < 59.0551) {
            pitch = -0.0000231316 * Math.pow(distance, 2)
                    - 0.0111474 * distance
                    + 1.28786;
        } else {
            pitch = -0.0000818672 * Math.pow(distance, 2)
                    + 0.00602864 * distance
                    + 0.972094;
        }

        // Telemetría para depuración
        dashboardTelemetry.addData("Velocity top (tps)", Top.getVelocity());
        dashboardTelemetry.addData("Velocity bottom (tps)", Down.getVelocity());
        dashboardTelemetry.addData("Shooter setpoint (tps)", shooterController.getSetPoint());
        dashboardTelemetry.addData("PID error top", shooterController.getPositionError());
        dashboardTelemetry.addData("Power top (applied)", Top.getPower());
        dashboardTelemetry.addData("Power bottom (applied)", Down.getPower());
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Top.setPower(0);
        Down.setPower(0);
        Intake.setPower(0);
        Index.setPower(0);
    }

    private double distanceWithTargetX(MecanumDrive drive, Telemetry telemetry) {
        double distance = ((-62 + offsetX) - drive.localizer.getPose().position.x) * 0.0254;
        telemetry.addData("Distance with target X", distance);
        return distance;
    }

    private double distanceWithTargetY(MecanumDrive drive, Telemetry telemetry, double allianceMult) {
        double distance = ((60 + offsetY) * allianceMult - drive.localizer.getPose().position.y) * 0.0254;
        telemetry.addData("Distance with target Y", distance);
        return distance;
    }

    private double clamp(double v, double min, double max) {
        if (v > max) return max;
        if (v < min) return min;
        return v;
    }
}