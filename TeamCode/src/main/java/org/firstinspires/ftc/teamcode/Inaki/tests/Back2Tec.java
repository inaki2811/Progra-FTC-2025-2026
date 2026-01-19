package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Index;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Yaw;

@Config
@TeleOp(name="Back2Tec", group="Test")
public class Back2Tec extends OpMode {
    private IntakeIO intakeIO;
    private Index index;
    private boolean indexStopped = false;
    private MecanumDrive drive;
    private Yaw yaw;
    private ShooterIO shooterIO;


    @Override
    public void init() {

        intakeIO = new IntakeIO();
        intakeIO.init(hardwareMap);
        shooterIO = new ShooterIO();
        shooterIO.init(hardwareMap);

        index = new Index();
        index.init(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        yaw = new Yaw();
        yaw.init(hardwareMap);
    }

    @Override
    public void loop() {


        if (gamepad1.right_trigger > 0.1) {
            intakeIO.setVelocity(-400);
            index.setVelocity(-1);
            boolean color = index.isBallDetected();
            telemetry.addData("Color detectado", color);

            if (color) {
                index.setVelocity(0);
                indexStopped = true;

            }
        } else {

            index.setVelocity(0);
            indexStopped = true;
            intakeIO.setVelocity(0);
            telemetry.addData("Index detenido", indexStopped);
            telemetry.addData("IntakeIO detenido", indexStopped);
            telemetry.update();
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        // === MOVIMIENTO ===
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forward, strafe),
                turn
        ));

        // Actualizar odometría
        drive.updatePoseEstimate();
        Pose2d odomPose = drive.localizer.getPose();

        telemetry.addData("Index detenido", indexStopped);
        telemetry.update();

        double input = gamepad2.left_stick_x;
        double targetAngle = input * 60; // ±30 grados

        yaw.setTargetAngleDeg(targetAngle);

        telemetry.addData("Joystick Input", input);
        telemetry.addData("Turret Angle (deg)", yaw.getTurretAngleDeg());
        telemetry.addData("Camera Yaw (rad)", yaw.getCameraYawRad());
        telemetry.update();

        if (gamepad2.right_trigger > 0.1){
            shooterIO.setTargetPower(0);
            shooterIO.update();
            shooterIO.setTargetVelocity(400);
            shooterIO.update();

        }else{
            shooterIO.setTargetPower(0);
            shooterIO.update();
        }

    }

    @Override
    public void stop() {
        intakeIO.setVelocity(0);
        index.setVelocity(0);
        yaw.stop();
    }
}