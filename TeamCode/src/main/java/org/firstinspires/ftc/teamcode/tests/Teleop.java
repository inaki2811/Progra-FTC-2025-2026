package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.RobotState;
import org.firstinspires.ftc.teamcode.core.SubsystemManager;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;


@TeleOp(name="Teleop", group="Regional")
public class Teleop extends OpMode {

    private MecanumDrive drive;
    private SubsystemManager subsystemManager;

    private boolean alreadyPressedA = false;
    private boolean alreadyPressedB = false;
    private boolean alreadyPressedX = false;
    private boolean alreadyPressedY = false;

    private ShooterIO shooter;
    private IntakeIO intake;
    private VisionIO vision;

    private boolean initialPoseSet = false;

    private boolean allianceDecided = false;



    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        subsystemManager = new SubsystemManager(hardwareMap, telemetry);
        intake = new IntakeIO(hardwareMap);
        shooter = new ShooterIO(hardwareMap);

        vision = new VisionIO(hardwareMap, shooter, telemetry);
        vision.resume();
        initialPoseSet = false;


        allianceDecided = false;

        telemetry.addLine("Test RR + Mecanum listo");
        telemetry.addData("status", "init complete");
        telemetry.update();

    }

    @Override
    public void loop() {


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


        subsystemManager.periodic(drive,() -> vision.getTagBySpecificId(20), new TelemetryPacket(), -1);

        // === LECTURA DE STICKS ===
        double driveY = -gamepad1.left_stick_x;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_y;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        Pose2d pose = drive.localizer.getPose();

        // === ACTUALIZA POSE ===
        drive.updatePoseEstimate();
        double heading = -pose.heading.toDouble() - Math.toRadians(180);

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
            drive.localizer.setPose(new Pose2d(-70, -59,  -Math.PI / 2));
        }


        // === TELEMETRÍA ===
        telemetry.addData("Estado:", RobotState.values());
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));


        telemetry.update();
    }

    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}