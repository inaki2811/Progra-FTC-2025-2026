package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionIO {
    private static final boolean USE_WEBCAM = true;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private final ShooterIO shooterIO;

    private final Position cameraPosition;

    private volatile Pose2dSimple lastRobotPose = null;

    private List <AprilTagDetection> detectedTags = new ArrayList<>();
    private final Telemetry telemetry;

    public VisionIO(HardwareMap hw, ShooterIO shooterIO, Telemetry telemetry) {
        this.shooterIO = shooterIO;

        // Camera pose en metros y orientaciones en radianes
        this.cameraPosition = new Position(DistanceUnit.INCH, 0, 6.5,2 , 0);

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-90), 0, 0);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hw.get(WebcamName.class, "Logi1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT);
        }

        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();

        this.telemetry = telemetry;
    }

    /**
     * update(): no bloqueante. Toma la primera detección si existe,
     * corrige heading con shooterIO.getYaw() y guarda pose en metros/rad.
     */
    public void update() {
        detectedTags = aprilTagProcessor.getDetections();

        if (detectedTags == null || detectedTags.isEmpty()) return;

        AprilTagDetection d = detectedTags.get(0);

        double camX = d.robotPose.getPosition().x;
        double camY = d.robotPose.getPosition().y;

        double camYaw = d.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        double turretYaw = shooterIO.getYaw();

        double offsetY = cameraPosition.y;
        double offsetX = cameraPosition.x;

        double ry = Math.sin(turretYaw) * offsetY + Math.cos(turretYaw) * offsetY;
        double rx = Math.cos(turretYaw) * offsetX - Math.sin(turretYaw) * offsetY;

        double robotX = camX - rx;
        double robotY = camY - ry;

        double correctedHeading = normalize(camYaw + turretYaw);

        lastRobotPose = new Pose2dSimple(robotX, robotY, correctedHeading);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {return;}

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }

        return null;
    }

    public void resume() {
        try { visionPortal.resumeStreaming(); } catch (Exception ignored) {}
    }

    public void pause() {
        try { visionPortal.stopStreaming(); } catch (Exception ignored) {}
    }

    public void close() {
        try { visionPortal.close(); } catch (Exception ignored) {}
    }

    public Pose2dSimple getLastRobotPose() { return lastRobotPose; }

    private double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public static class Pose2dSimple {
        public final double x;
        public final double y;
        public final double heading;
        public Pose2dSimple(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
        @Override public String toString() { return "Pose2dSimple{x=" + x + ", y=" + y + ", heading=" + heading + "}"; }
    }
}