package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class Shoot {

    private final ShooterIO shooterIO;
    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;
    private final Supplier<AprilTagDetection> tagDetection;
    private final Telemetry telemetry;

    private double targetVelocity = 0;
    private double velOffset;

    // Debug counters
    private int prepareCallCount = 0;
    private long prepareStartTime = 0;

    public Shoot(
            ShooterIO shooterIO,
            Supplier<Double> distanceWithTargetX,
            Supplier<Double> distanceWithTargetY,
            Supplier<Double> botYaw,
            Supplier<AprilTagDetection> tagDetection,
            double velOffset,
            Telemetry telemetry
    ) {
        this.shooterIO = shooterIO;
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;
        this.tagDetection = tagDetection;
        this.velOffset = velOffset;
        this.telemetry = telemetry;
    }

    /* ------------------ VELOCITY ------------------ */

    public void setVel(double velocity) {
        targetVelocity = velocity;
        shooterIO.setPoint(velocity + velOffset);
        shooterIO.setVel();
    }

    public double getVel() {
        return shooterIO.getVelocity();
    }

    public boolean atVelocity() {
        double current = shooterIO.getVelocity();
        double delta = Math.abs(current - targetVelocity);
        return delta < 20;
    }

    public Action spinUp(double velocity) {
        return packet -> {
            setVel(velocity);
            return true;
        };
    }

    public Action stop() {
        return packet -> {
            targetVelocity = 0;
            shooterIO.setPwr(0);
            prepareCallCount = 0;
            prepareStartTime = 0;
            telemetry.addData("🛑 Shooter", "STOPPED");
            return true;
        };
    }

    public Action prepareVelocity() {
        return packet -> {
            // Primera vez que se llama esta acción
            if (prepareCallCount == 0) {
                prepareStartTime = System.currentTimeMillis();
            }
            prepareCallCount++;

            double distance = getDistance();
            targetVelocity = 550;
            shooterIO.setPwr(1);

            double currentVel = shooterIO.getVelocity();
            boolean ready = atVelocity();
            long elapsed = System.currentTimeMillis() - prepareStartTime;

            telemetry.addData("━━━━━ PREPARE VEL ━━━━━", "");
            telemetry.addData("  Call Count", prepareCallCount);
            telemetry.addData("  Elapsed (ms)", elapsed);
            telemetry.addData("  Distance", String.format("%.2f m", distance));
            telemetry.addData("  Current Vel", String.format("%.1f", currentVel));
            telemetry.addData("  Target Vel", targetVelocity);
            telemetry.addData("  Delta", String.format("%.1f", Math.abs(currentVel - targetVelocity)));
            telemetry.addData("  Ready", ready ? "✅ YES" : "❌ NO");
            telemetry.addData("  Returning", ready);

            if (ready) {
                telemetry.addData("🎯 VELOCITY READY!", "Moving to shoot");
                prepareCallCount = 0; // Reset para próxima vez
            }

            return ready;
        };
    }

    /* ------------------ TRACKING ------------------ */

    public Action trackingYaw() {
        return packet -> {
            double yaw = Math.atan2(distanceWithTargetY.get(), distanceWithTargetX.get())
                    - botYaw.get();

            shooterIO.setYaw(yaw);

            boolean ready = atVelocity();
            telemetry.addData("  Yaw tracking", String.format("%.1f°", Math.toDegrees(yaw)));

            return ready;
        };
    }

    public Action trackingHood() {
        return packet -> {
            double distance = getDistance();
            double pitch;

            if (distance < 59.0) {
                pitch = -0.0000231316 * Math.pow(distance, 2)
                        - 0.0111474 * distance
                        + 1.28786;
            } else {
                pitch = -0.0000818672 * Math.pow(distance, 2)
                        + 0.00602864 * distance
                        + 0.972094;
            }

            shooterIO.setHood(pitch);

            boolean ready = atVelocity();
            telemetry.addData("  Hood pitch", String.format("%.2f", pitch));

            return ready;
        };
    }

    /* ------------------ UTILS ------------------ */

    public double getDistance() {
        Double x = distanceWithTargetX.get();
        Double y = distanceWithTargetY.get();

        if (x == null || y == null) {
            telemetry.addData("⚠️ WARNING", "Distance null!");
            return 0.0;
        }

        return Math.hypot(x, y);
    }
}