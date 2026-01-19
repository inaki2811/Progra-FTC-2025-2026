package org.firstinspires.ftc.teamcode.Inaki.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Inaki.core.Subsystem;

import java.util.ArrayList;

public class Vision implements Subsystem {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection tagDetection;
    private KalmanFilter kalman;
    private Pose2d lastPoseEstimate;

    @Override
    public void init(HardwareMap hwMap) {
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 4, 8, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)    // formato estable
                .setCameraResolution(new Size(640, 480))
                .build();

        kalman = new KalmanFilter();
        lastPoseEstimate = new Pose2d(0,0,0);
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public AprilTagDetection getBestDetection() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        AprilTagDetection best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (d.ftcPose == null) continue;
            double score = d.ftcPose.z + Math.abs(d.ftcPose.x) * 0.5;
            if (score < bestScore) {
                bestScore = score;
                best = d;
            }
        }
        return best;
    }

    /** Pose de cámara */
    public Pose2d getCameraPose() {
        AprilTagDetection best = getBestDetection();
        if (best == null || best.ftcPose == null) return null;

        double x = best.ftcPose.x;
        double y = best.ftcPose.y;
        double heading = Math.toRadians(best.ftcPose.yaw);

        return new Pose2d(x, y, heading);

    }

    /** Fusión con Kalman */
    public Pose2d fuseWithOdom(Pose2d odoPose) {
        Pose2d camPose = getCameraPose();
        if (camPose != null) {
            lastPoseEstimate = kalman.update(odoPose, camPose);
        } else {
            lastPoseEstimate = kalman.predict(odoPose);
        }
        return lastPoseEstimate;
    }

    public Pose2d getPoseEstimate() {
        return lastPoseEstimate;
    }

    @Override public void update() {}
    @Override public void stop() { if (visionPortal != null) visionPortal.stopStreaming(); }
}