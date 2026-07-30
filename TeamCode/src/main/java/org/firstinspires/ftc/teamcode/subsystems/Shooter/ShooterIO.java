package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


public class ShooterIO {

    /// ================= SHOOTER =================
    private final DcMotorEx shooterUp, shooterDown;

    private double targetVelocity = 0.0;
    private static final double VELOCITY_TOLERANCE = 75.0;

    // Ajustables desde Dashboard
    public static PIDFCoefficients shooterCoeffs =
            new PIDFCoefficients(60, 0, 6, 13); // valores típicos para flywheel FTC

    /// ================= YAW =================
    private final Servo leftServo, rightServo;
    public static double posMin = 0.0;
    public static double posMax = 1.0;
    public static double maxAngleDeg = 60.0;

    private double currentAngleDeg = 0.0;

    /// ================= HOOD =================
    private final Servo hood;

    public ShooterIO(HardwareMap hwMap) {

        // ---- Shooter motors ----
        shooterUp = hwMap.get(DcMotorEx.class, "launcherTop");
        shooterDown = hwMap.get(DcMotorEx.class, "launcherBottom");

        shooterUp.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterUp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterCoeffs);
        shooterDown.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterCoeffs);

        // ---- Yaw ----
        leftServo  = hwMap.get(Servo.class, "yawLeft");
        rightServo = hwMap.get(Servo.class, "yawRight");

        // ---- Hood ----
        hood = hwMap.get(Servo.class, "hood");
    }

    // ================= YAW =================

    public void setYaw(double angleDeg) {
        double clamped = clampAngle(angleDeg);
        currentAngleDeg = clamped;

        double pos = angleToServoPos(clamped);
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public double getYaw() {
        return currentAngleDeg;
    }

    private double clampAngle(double angleDeg) {
        if (angleDeg > maxAngleDeg) return maxAngleDeg;
        if (angleDeg < -maxAngleDeg) return -maxAngleDeg;
        return angleDeg;
    }

    private double angleToServoPos(double angleDeg) {
        double t = (angleDeg + maxAngleDeg) / (2.0 * maxAngleDeg); // 0..1
        return posMin + t * (posMax - posMin);
    }

    // ================= HOOD =================

    public void setHood(double pos) {
        hood.setPosition(pos);
    }

    public double getHood() {
        return hood.getPosition();
    }

    // ================= SHOOTER =================

    /** Setea la referencia */
    public void setPoint(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
    }

    /** Aplica la referencia (llamar en loop / Action) */
    public void setVel() {
        shooterUp.setVelocity(targetVelocity);
        shooterDown.setVelocity(targetVelocity);
    }

    public void stop() {
        shooterUp.setVelocity(0);
        shooterDown.setVelocity(0);
        targetVelocity = 0;
    }

    public double getVelocity() {
        return (shooterUp.getVelocity() + shooterDown.getVelocity()) / 2.0;
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - targetVelocity) <= VELOCITY_TOLERANCE;
    }

    /** Solo para testing manual
    public void setPwr(double power) {
        shooterUp.setPower(power);
        shooterDown.setPower(power);
    }*/
    public void setPwr(double power) {
        shooterUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterUp.setPower(power);
        shooterDown.setPower(power);
    }
}