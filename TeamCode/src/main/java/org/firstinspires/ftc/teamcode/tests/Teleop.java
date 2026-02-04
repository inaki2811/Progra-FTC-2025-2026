package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@TeleOp(name="Teleop", group="Regional")
public class Teleop extends OpMode {

    private MecanumDrive drive;
    private ShooterIO shooterIO;
    private IntakeIO intakeIO;

    // ================= SHOOTER SEQUENCE =================

    private enum ShootRoutineState {
        IDLE,
        WAIT_READY,    // Waiting for velocity to reach target
        FEEDING        // Actively feeding the ball
    }

    private ShootRoutineState shootState = ShootRoutineState.IDLE;
    private ElapsedTime feedTimer = new ElapsedTime();

    private boolean shooterOn = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    // Ajustables desde Dashboard
    public static double SHOOT_READY_VEL = 550;
    public static double VEL_TOLERANCE = 20; // Tolerance below target velocity
    public static double MIN_FEED_TIME = 0.3; // Minimum time to ensure ball is fed
    public static double MAX_FEED_TIME = 3.0; // Maximum time before timeout
    public static double MANUAL_INTAKE_POWER = 0.7; // Power for manual intake/index

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        shooterIO = new ShooterIO(hardwareMap);
        intakeIO = new IntakeIO(hardwareMap);

        telemetry.addLine("TeleOp listo");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        double driveY = -gamepad1.left_stick_x;
        double driveX = -gamepad1.left_stick_y;
        double turn   = -gamepad1.right_stick_x;

        double heading = -pose.heading.toDouble() - Math.PI;

        double rotatedX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
        double rotatedY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(rotatedX, rotatedY), turn)
        );

        // ================= MANUAL INTAKE/INDEX CONTROL =================

        boolean rb = gamepad1.right_bumper;

        // ================= START SHOOT ROUTINE =================

        boolean lb = gamepad1.left_bumper;

        if (lb && !lastLB && shootState == ShootRoutineState.IDLE) {
            shooterOn = true;
            shootState = ShootRoutineState.WAIT_READY;
        }

        lastLB = lb;
        lastRB = rb;

        // ================= SHOOTER POWER =================

        shooterIO.setPwr(shooterOn ? 1.0 : 0.0);

        // ================= SHOOT STATE MACHINE =================

        double currentVel = shooterIO.getVelocity();
        double threshold = SHOOT_READY_VEL - VEL_TOLERANCE;

        switch (shootState) {

            case IDLE:
                // Manual control with right bumper when not shooting
                if (rb) {
                    intakeIO.setPwrIntake(MANUAL_INTAKE_POWER);
                    intakeIO.setPwrIndex(MANUAL_INTAKE_POWER);
                } else {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                }
                break;

            case WAIT_READY:
                intakeIO.setPwrIntake(0.0);
                intakeIO.setPwrIndex(0.0);

                // Wait for velocity to reach threshold (target - 20)
                if (currentVel >= threshold) {
                    feedTimer.reset();
                    shootState = ShootRoutineState.FEEDING;
                }
                break;

            case FEEDING:
                // Only feed while velocity is within range (threshold to target)
                if (currentVel >= threshold && currentVel <= SHOOT_READY_VEL) {
                    intakeIO.setPwrIntake(1.0);
                    intakeIO.setPwrIndex(1.0);
                } else {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                }

                // Once velocity drops below threshold AND minimum feed time has passed, shot is complete
                if (currentVel < threshold && feedTimer.seconds() >= MIN_FEED_TIME) {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                    shooterOn = false;
                    shootState = ShootRoutineState.IDLE;
                }

                // Timeout: if feeding for too long without detecting a shot, stop
                if (feedTimer.seconds() >= MAX_FEED_TIME) {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                    shooterOn = false;
                    shootState = ShootRoutineState.IDLE;
                }
                break;
        }

        // ================= TELEMETRY =================

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Shooter Vel", currentVel);
        telemetry.addData("Threshold", threshold);
        telemetry.addData("Target Vel", SHOOT_READY_VEL);
        telemetry.addData("Feed Time", feedTimer.seconds());
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Manual Intake (RB)", rb);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0,0), 0)
        );
        // Ensure everything is stopped
        intakeIO.setPwrIntake(0.0);
        intakeIO.setPwrIndex(0.0);
        shooterIO.setPwr(0.0);
    }
}