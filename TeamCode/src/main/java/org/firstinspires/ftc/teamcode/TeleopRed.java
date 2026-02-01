package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.RobotState;
import org.firstinspires.ftc.teamcode.core.SubsystemManager;
import org.firstinspires.ftc.teamcode.subsystems.Vision.AllianceDetector;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO.Pose2dSimple;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterIO;


import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp(name="TeleopBlue", group="Regional")
public class TeleopRed extends OpMode {

    private MecanumDrive drive;
    private SubsystemManager subsystemManager;

    private boolean alreadyPressedA = false;
    private boolean alreadyPressedB = false;
    private boolean alreadyPressedX = false;
    private boolean alreadyPressedY = false;

    private ShooterIO shooter;
    private VisionIO vision;
    private boolean initialPoseSet = false;

    private AllianceDetector allianceDetector;
    private boolean allianceDecided = false;
    private Servo hammerShooter;



    @Override
    public void init() {
        hammerShooter = hardwareMap.get(Servo.class, "hammerS");
        drive = new MecanumDrive(hardwareMap, AutonomousBlue.lastPose.get());
        subsystemManager = new SubsystemManager(hardwareMap, telemetry);

        shooter = new ShooterIO(hardwareMap);
        vision = new VisionIO(hardwareMap, shooter, telemetry);
        vision.resume();

        initialPoseSet = false;

        allianceDetector = new AllianceDetector();
        allianceDetector.fieldCenterY = 0.0;

        allianceDetector.thresholdMeters = 0.3;    // zona segura
        allianceDetector.maxSamples = 3;           //muestras

        allianceDecided = false;

        telemetry.addLine("Test RR + Mecanum listo");
        telemetry.addData("status", "init complete");
        telemetry.update();

    }

    @Override
    public void loop() {
        subsystemManager.periodic(drive, () -> vision.getTagBySpecificId(20), new TelemetryPacket(), 1);

        vision.update();
        vision.displayDetectionTelemetry(vision.getTagBySpecificId(20));
        Pose2dSimple vp = vision.getLastRobotPose();
        /*
        // === DETECCIÓN DE ALIANZA ===
        if (!allianceDecided){
            if (allianceDetector.processPose(vp)){
                allianceDecided = true;
                if (vp != null){
                    drive.localizer.setPose(new Pose2d(vp.x, vp.y, vp.heading));
                }

                try { vision.pause(); } catch (Exception ignored) {}

            }
        }
        */

        // === LECTURA DE STICKS ===
        double driveY = -gamepad1.left_stick_x;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_y;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        Pose2d pose = drive.localizer.getPose();

        // === ACTUALIZA POSE ===
        drive.updatePoseEstimate();
        double heading = pose.heading.toDouble() - Math.toRadians(180);

        // === CONVERSIÓN FIELD ORIENTED ===
        double rotatedX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
        double rotatedY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

        // === MOVIMIENTO ===
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotatedX, rotatedY),
                turn
        ));

        if (gamepad2.a && !alreadyPressedA) {
            subsystemManager.setState(RobotState.INTAKE);

            alreadyPressedA = true;
        } else {
            alreadyPressedA = false;
        }

        if (gamepad2.b && !alreadyPressedB) {
            subsystemManager.setState(RobotState.SHOOT);
            alreadyPressedB = true;
        } else {
            alreadyPressedB = false;
        }


        if (gamepad2.x && !alreadyPressedX) {
            subsystemManager.setState(RobotState.STOP);
            alreadyPressedX = true;
        } else {
            alreadyPressedX = false;
        }

        if (gamepad1.dpad_down) {
            drive.localizer.setPose(new Pose2d(-70, 59,  Math.PI / 2));
        }


        // === TELEMETRÍA ===
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("HammerPosition", hammerShooter.getPosition());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("alliance", allianceDecided ? allianceDetector.getAlliance().toString() : "pending");
        telemetry.addData("samples", allianceDetector.getSamples());
        telemetry.addData("red", allianceDetector.getRedCount());
        telemetry.addData("blue", allianceDetector.getBlueCount());

        telemetry.update();
    }

    @Override
    public void stop() {
        try {vision.close();} catch (Exception ignored) {}
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}