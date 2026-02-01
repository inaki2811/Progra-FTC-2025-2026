package org.firstinspires.ftc.teamcode.Inaki.core;

import com.acmerobotics.roadrunner.Pose2d;

public class KalmanFilter {
    private Pose2d stateEstimate;

    private double alphaOdo = 0.7;   // peso de odometría
    private double alphaCam = 0.3;   // peso de cámara

    public KalmanFilter() {
        stateEstimate = new Pose2d(0,0,0);
    }

    // Predicción solo con odometría
    public Pose2d predict(Pose2d odoPose) {
        stateEstimate = odoPose;
        return stateEstimate;
    }

    public Pose2d update(Pose2d odoPose, Pose2d camPose) {
        if (camPose == null) {
            stateEstimate = odoPose;
        } else {
            double x = alphaOdo * odoPose.position.x + alphaCam * camPose.position.x;
            double y = alphaOdo * odoPose.position.y + alphaCam * camPose.position.y;
            double heading = alphaOdo * odoPose.heading.toDouble() + alphaCam * camPose.heading.toDouble();

            stateEstimate = new Pose2d(x, y, heading);
        }
        return stateEstimate;
    }

    public Pose2d getEstimate() {
        return stateEstimate;
    }

    public void setWeights(double alphaOdo, double alphaCam) {
        this.alphaOdo = alphaOdo;
        this.alphaCam = alphaCam;
    }
}