package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Yaw;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.KalmanFilter;

@TeleOp(name="TestPoseTrackingKalman", group="Test")
public class TestPoseTrackingKalman extends OpMode {
    private MecanumDrive drive;
    private Vision vision;
    private Yaw yawSubsystem;
    private KalmanFilter kalman;

    private Pose2d initPose = null;

    @Override
    public void init() {
        // Inicializar subsistemas
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        vision = new Vision();
        vision.init(hardwareMap);

        yawSubsystem = new Yaw();
        yawSubsystem.init(hardwareMap);

        kalman = new KalmanFilter();

        Pose2d camPose = vision.getCameraPose();
        if (camPose != null) {

            double camYaw = yawSubsystem.getCameraYawRad();
            initPose = new Pose2d(camPose.position.x, camPose.position.y, camYaw);
            drive.localizer.setPose(initPose);
            telemetry.addData("Pose inicial detectada", initPose);
        } else {
            initPose = new Pose2d(0,0,0);
            drive.localizer.setPose(initPose);
            telemetry.addLine("No se detectó AprilTag, usando (0,0,0)");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        // Actualizar odometría
        drive.updatePoseEstimate();
        Pose2d odomPose = drive.localizer.getPose();

        // Obtener pose de cámara (puede ser null)
        Pose2d camPose = vision.getCameraPose();

        // Fusionar con Kalman (si tu Kalman no acepta null, maneja aquí)
        Pose2d fusedPose;
        if (camPose != null) {
            fusedPose = kalman.update(odomPose, camPose);
        } else {
            // Si no hay lectura de cámara, usa solo odometría o el propio filtro
            fusedPose = kalman.update(odomPose, null); // solo si update acepta null
            // o simplemente:
            // fusedPose = odomPose;
        }
        drive.localizer.setPose(fusedPose);

        // === MOVIMIENTO ===
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forward, strafe),
                turn
        ));

        // Telemetry con etiquetas únicas y comprobaciones de null
        telemetry.addData("Odom X", odomPose.position.x);
        telemetry.addData("Odom Y", odomPose.position.y);
        telemetry.addData("Odom Heading", odomPose.heading);

        if (camPose != null) {
            telemetry.addData("Cam X", camPose.position.x);
            telemetry.addData("Cam Y", camPose.position.y);
            telemetry.addData("Cam Heading", camPose.heading);
        } else {
            telemetry.addLine("Cam Pose: null (no AprilTag detectado)");
        }

        telemetry.addData("Fused X", fusedPose.position.x);
        telemetry.addData("Fused Y", fusedPose.position.y);
        telemetry.addData("Fused Heading", fusedPose.heading);

        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}