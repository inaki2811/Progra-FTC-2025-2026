package org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterIO {


    private final HardwareMap hwMap;

    /// SHOOTER
    private DcMotorEx shooterUp, shooterDown;
    private double targetVelocity = 0.0;
    private static final double VELOCITY_TOLERANCE = 50.0;
    public static double kA = 0.0007;
    public static double vel = 720;



    ///  YAW
    private Servo leftServo, rightServo;
    public static double posMin = 0.0;
    public static double posMax = 1.0;
    public static double maxAngleDeg = 60.0;
    private double currentPos = 0.5;
    private double currentAngleDeg = 0.0;


    ///  HOOD
    private Servo hood;


    public static final PIDFController shooterController = new PIDFController(0.09, 0.9, 0.000001, 0.0);

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            0.09, 0.9, 0.000001, 0.0
    );

    public  ShooterIO (HardwareMap hwMap) {
        this.hwMap = hwMap;

        ///Shooter
        shooterUp   = hwMap.get(DcMotorEx.class, "launcherTop");
        shooterDown = hwMap.get(DcMotorEx.class, "launcherBottom");
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterUp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterDown.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        shooterController.setCoefficients(shooterCoeffs);

        shooterController.setTolerance(30);

        double currentVelocity = shooterUp.getVelocity();

        vel = ( kA * vel) + shooterController.calculate(currentVelocity);

        vel = Math.max(-1, Math.min(vel, 1));
        if(Math.abs(shooterController.getSetPoint()) > 100){
            shooterUp.setPower(vel);
            shooterDown.setPower(vel);
        } else{
            shooterUp.setPower(0);
            shooterDown.setPower(0);
        }
    }

    public void setPoint(double setPoint){
        shooterController.setSetPoint(setPoint);
    }

    public void setPower(double power) {
        shooterDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterUp.setPower(power);
        shooterDown.setPower(power);

    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void setTargetPower(double power) {
        shooterUp.setPower(power);
        shooterDown.setPower(power);
    }

    public double getVelocity() {
        return (shooterUp.getVelocity() + shooterDown.getVelocity()) / 2;
    }
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

}