package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestYaw", group="Test")
public class TestYaw extends OpMode {
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

    @Override
    public void init() {

        leftServo  = hardwareMap.get(Servo.class, "yawLeft");
        rightServo = hardwareMap.get(Servo.class, "yawRight");

        setTargetAngleDeg(0.0);
    }

    @Override
    public void loop() {
        double input = gamepad1.right_stick_x;
        double targetAngle = input * 60; // ±30 grados

        setTargetAngleDeg(targetAngle);

        telemetry.addData("Joystick Input", input);
        telemetry.update();
    }

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

    @Override
    public void stop() {
        setTargetAngleDeg(0);
    }
}