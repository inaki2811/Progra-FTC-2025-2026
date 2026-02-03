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
        WAIT_READY,    // Waiting for velocity to reach target (first ball only)
        FEEDING,       // Actively feeding a ball
        WAIT_RECOVERY  // Waiting for velocity to recover after shot
    }

    private ShootRoutineState shootState = ShootRoutineState.IDLE;
    private ElapsedTime feedTimer = new ElapsedTime();

    private boolean shooterOn = false;
    private boolean lastLB = false;

    // Ajustables desde Dashboard
    public static double SHOOT_READY_VEL = 550;
    public static double VEL_TOLERANCE = 20; // Tolerance below target velocity
    public static double SECOND_BALL_BONUS = 50; // Extra velocity needed for second ball
    public static double MIN_FEED_TIME = 0.3; // Minimum time to ensure ball is fed
    public static int MAX_SHOTS = 3; // Number of balls to shoot

    private int shotCount = 0;

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

        // ================= START SHOOT ROUTINE =================

        boolean lb = gamepad1.left_bumper;

        if (lb && !lastLB && shootState == ShootRoutineState.IDLE) {
            shooterOn = true;
            shotCount = 0;
            shootState = ShootRoutineState.WAIT_READY;
        }

        lastLB = lb;

        // ================= SHOOTER POWER =================

        shooterIO.setPwr(shooterOn ? 1.0 : 0.0);

        // ================= SHOOT STATE MACHINE =================

        double currentVel = shooterIO.getVelocity();

        // Determine threshold based on which ball we're shooting
        double threshold;
        if (shotCount == 1) {
            // Second ball needs higher velocity (target + 30)
            threshold = SHOOT_READY_VEL + SECOND_BALL_BONUS;
        } else {
            // First and third balls use normal threshold (target - 20)
            threshold = SHOOT_READY_VEL - VEL_TOLERANCE;
        }

        switch (shootState) {

            case IDLE:
                intakeIO.setPwrIntake(0.0);
                intakeIO.setPwrIndex(0.0);
                shotCount = 0;
                break;

            case WAIT_READY:
                intakeIO.setPwrIntake(0.0);
                intakeIO.setPwrIndex(0.0);

                // Wait for velocity to reach threshold
                if (currentVel >= threshold) {
                    feedTimer.reset();
                    shootState = ShootRoutineState.FEEDING;
                }
                break;

            case FEEDING:
                // Feed ball while velocity is above threshold
                if (currentVel >= threshold) {
                    intakeIO.setPwrIntake(1.0);
                    intakeIO.setPwrIndex(1.0);
                } else {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                }

                // Once velocity drops below threshold AND minimum feed time has passed
                if (currentVel < threshold && feedTimer.seconds() >= MIN_FEED_TIME) {
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                    shotCount++;
                    shootState = ShootRoutineState.WAIT_RECOVERY;
                }
                break;

            case WAIT_RECOVERY:
                intakeIO.setPwrIntake(0.0);
                intakeIO.setPwrIndex(0.0);

                // Check if we've shot all balls first
                if (shotCount >= MAX_SHOTS) {
                    shooterOn = false;
                    shootState = ShootRoutineState.IDLE;
                    // Explicitly stop everything
                    intakeIO.setPwrIntake(0.0);
                    intakeIO.setPwrIndex(0.0);
                    shooterIO.setPwr(0.0);
                }
                // Wait for velocity to recover to threshold for next ball
                else if (currentVel >= threshold) {
                    // Go directly to FEEDING for subsequent balls
                    feedTimer.reset();
                    shootState = ShootRoutineState.FEEDING;
                }
                break;
        }

        // ================= TELEMETRY =================

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Shot Count", shotCount);
        telemetry.addData("Shooter Vel", currentVel);
        telemetry.addData("Current Threshold", threshold);
        telemetry.addData("Feed Time", feedTimer.seconds());
        telemetry.addData("Shooter On", shooterOn);
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