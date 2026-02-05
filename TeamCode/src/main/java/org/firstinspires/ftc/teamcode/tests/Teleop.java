package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.subsystems.Vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Vision.AllianceDetector;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionIO;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@TeleOp(name="Teleop", group="Regional")
public class Teleop extends OpMode {

    private MecanumDrive drive;
    private SubsystemManager subsystemManager;
    private boolean alreadyPressedA = false;
    private boolean alreadyPressedB = false;
    private boolean alreadyPressedX = false;
    private boolean alreadyPressedY = false;
    private VisionIO vision;
    private AllianceDetector allianceDetector;
    private boolean allianceDecided = false;
    private int allianceMult = -1; // azul = -1, rojo = +1
    private ShooterIO shooterIO;
    private IntakeIO intakeIO;

    public static double intake, index;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        IntakeIO intakeIO = new IntakeIO(hardwareMap);
        ShooterIO shooterIO = new ShooterIO(hardwareMap);
        vision = new VisionIO(hardwareMap, shooterIO, telemetry);
        allianceDetector = new AllianceDetector();
        vision.resume();

        subsystemManager = new SubsystemManager(hardwareMap, telemetry, drive, () -> vision.getTagBySpecificId(20));

        telemetry.addLine("inicio de teleop");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        subsystemManager.periodic(packet);



        // === DETECCIÓN DE ALIANZA ===

        vision.update();

        if (!allianceDecided) {
            VisionIO.Pose2dSimple vp = vision.getLastRobotPose();

            if (allianceDetector.processPose(vp)) {
                Alliance alliance = allianceDetector.getAlliance();

                if (alliance == Alliance.RED) {
                    allianceMult = 1;
                } else if (alliance == Alliance.BLUE) {
                    allianceMult = -1;
                }

                if (vp != null) {
                    drive.localizer.setPose(
                            new Pose2d(vp.x, vp.y, vp.heading)
                    );
                }

                vision.pause();
                allianceDecided = true;
            }
        }

        // === LECTURA DE STICKS y ACTUALIZA POSE===

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        double driveY = -gamepad1.left_stick_x;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_y;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        double heading = -pose.heading.toDouble() - Math.toRadians(180);

        // === CONVERSIÓN FIELD ORIENTED ===

        double rotatedX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
        double rotatedY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotatedX, rotatedY), turn));

        // === MAQUINA DE ESTADOS ===

        if (gamepad1.a && !alreadyPressedA) {
            subsystemManager.setState(RobotState.INTAKE);
            alreadyPressedA = true;
        } else {
            alreadyPressedA = false;
        }

        if (gamepad1.b && !alreadyPressedB) {
            subsystemManager.setState(RobotState.SHOOT);
            alreadyPressedB = true;
        } else {
            alreadyPressedB = false;
        }

        if (gamepad1.x && !alreadyPressedX) {
            subsystemManager.setState(RobotState.STOP);
            alreadyPressedX = true;
        } else {
            alreadyPressedX = false;
        }

        if (gamepad1.dpad_down) {
            drive.localizer.setPose(new Pose2d(-70, -59,  -Math.PI / 2));
        }

        if (gamepad1.dpad_left){
            vision.resume();
            VisionIO.Pose2dSimple vp = vision.getLastRobotPose();

        } else if (gamepad1.dpad_right) {
            vision.close();

        }


        // === TELEMETRÍA ===
        telemetry.addData("Alliance decided", allianceDecided);
        telemetry.addData("Alliance", allianceDetector.getAlliance());
        telemetry.addData("Estado:", RobotState.values());
        telemetry.addData("Pose X", pose.position.x);
        telemetry.addData("Pose Y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }



    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}