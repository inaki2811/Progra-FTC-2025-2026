package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterIO {


    private final HardwareMap hwMap;

    /// SHOOTER
    private DcMotorEx shooterUp, shooterDown;
    private double targetVelocity = 0.0;
    private static final double VELOCITY_TOLERANCE = 50.0;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            1, 10, 0.000001, 0.0
    );

    ///  YAW
    private Servo leftServo, rightServo;
    public static double posMin = 0.0;
    public static double posMax = 1.0;
    public static double maxAngleDeg = 60.0;
    private double currentPos = 0.5;
    private double currentAngleDeg = 0.0;

    ///  HOOD
    private Servo hood;

    public  ShooterIO (HardwareMap hwMap) {
        this.hwMap = hwMap;

        ///Shooter
        shooterUp   = hwMap.get(DcMotorEx.class, "launcherTop");
        shooterDown = hwMap.get(DcMotorEx.class, "launcherBottom");
        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterUp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterCoeffs);
        shooterDown.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterCoeffs);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);

        ///Yaw
        leftServo  = hwMap.get(Servo.class, "yawLeft");
        rightServo = hwMap.get(Servo.class, "yawRight");

        ///Hood
        hood = hwMap.get(Servo.class, "hood");

    }

    /// YAW ///
    public void setYaw(double angleDeg){

        double clamped = clampAngle(angleDeg);
        currentAngleDeg = clamped;

        // Convertir ángulo a posición servo
        double pos = angleToServoPos(clamped);
        currentPos = pos;

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


    /// HOOD ///
    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public double getHood(){
        return hood.getPosition();
    }


    /// SHOOTER ///
    public void setVel(){
        shooterUp.setVelocity(targetVelocity);
        shooterDown.setVelocity(targetVelocity);
    }

    public void setPoint(double setPoint){
        targetVelocity = setPoint;
    }


    public double getVelocity() {
        return (shooterUp.getVelocity() + shooterDown.getVelocity()) / 2;
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    public void setPwr (double power){
        shooterUp.setPower(power);
        shooterDown.setPower(power);

    }

}