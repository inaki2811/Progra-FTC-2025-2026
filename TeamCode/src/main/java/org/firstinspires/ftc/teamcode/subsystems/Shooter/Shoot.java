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

    public boolean atVelocity() {
        return Math.abs(shooterIO.getVelocity() - targetVelocity) < 50;
    }

    public Action spinUp(double velocity) {
        return packet -> {
            setVel(velocity);
            return true; // acción instantánea
        };
    }

    public Action stop() {
        return packet -> {
            setVel(0);
            return true;
        };
    }

    public Action prepareVelocity() {
        return packet -> {
            double distance = getDistance();

            if (distance < 59.0) {
                setVel(600);
            } else {
                setVel(700);
            }

            return atVelocity(); // termina cuando ya está listo
        };
    }

    /* ------------------ TRACKING ------------------ */

    public Action trackingYaw() {
        return packet -> {
            double yaw =
                    Math.atan2(distanceWithTargetY.get(), distanceWithTargetX.get())
                            - botYaw.get();

            shooterIO.setYaw(yaw);
            return false; // corre continuamente
        };
    }

    public Action trackingHood() {
        return packet -> {
            double distance = getDistance();
            double pitch;

            if (distance < 59.0) {
                pitch =
                        -0.0000231316 * Math.pow(distance, 2)
                                - 0.0111474 * distance
                                + 1.28786;
            } else {
                pitch =
                        -0.0000818672 * Math.pow(distance, 2)
                                + 0.00602864 * distance
                                + 0.972094;
            }

            shooterIO.setHood(pitch);
            return false;
        };
    }

    /* ------------------ UTILS ------------------ */

    public double getDistance() {
        return Math.hypot(distanceWithTargetX.get(), distanceWithTargetY.get());
    }
}