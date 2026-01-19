package org.firstinspires.ftc.teamcode.Inaki.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Inaki.core.Subsystem;

@Config
public class Yaw implements Subsystem {
    private Servo leftServo, rightServo;

    // Posiciones mínimas y máximas (ajustables en Dashboard)
    public static double posMin = 0.0;
    public static double posMax = 1.0;

    // Rango real medido: ±60°
    public static double maxAngleDeg = 60.0;

    // Relación helicoidal (gear ratio)
    public static double gearRatio = 2.45;

    // Offset de montaje de la cámara
    public static double mountOffsetCamDeg = 0.0;

    // Última posición mandada
    private double currentPos = 0.5; // centro (frente)
    private double currentAngleDeg = 0.0; // frente = 0°

    public void init(HardwareMap hwMap) {
        leftServo  = hwMap.get(Servo.class, "yawLeft");
        rightServo = hwMap.get(Servo.class, "yawRight");

        setTargetAngleDeg(0.0);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {
        setTargetAngleDeg(0.0);
    }

    // ----------------- Control por ángulo -----------------
    public void setTargetAngleDeg(double angleDeg) {
        // Limitar al rango físico ±60°
        double clamped = clampAngle(angleDeg);
        currentAngleDeg = clamped;

        // Convertir ángulo a posición servo
        double pos = angleToServoPos(clamped);
        currentPos = pos;

        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public double getTurretAngleDeg() {
        return currentAngleDeg;
    }

    public double getCameraYawRad() {
        double turretYawRad = Math.toRadians(currentAngleDeg);
        return turretYawRad * gearRatio + Math.toRadians(mountOffsetCamDeg);
    }

    // ----------------- Utilidades -----------------
    private double clampAngle(double angleDeg) {
        if (angleDeg > maxAngleDeg) return maxAngleDeg;
        if (angleDeg < -maxAngleDeg) return -maxAngleDeg;
        return angleDeg;
    }

    private double angleToServoPos(double angleDeg) {
        // Mapear -60° → 0.0, 0° → 0.5, +60° → 1.0
        double t = (angleDeg + maxAngleDeg) / (2.0 * maxAngleDeg); // 0..1
        return posMin + t * (posMax - posMin);
    }

    public double getLeftServoPos() { return leftServo.getPosition(); }
    public double getRightServoPos() { return rightServo.getPosition(); }
    public double getCurrentPos() { return currentPos; }
}