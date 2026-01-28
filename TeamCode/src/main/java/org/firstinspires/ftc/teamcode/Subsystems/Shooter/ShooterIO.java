package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Test.TuneShooterVelocity.kA;
import static org.firstinspires.ftc.teamcode.Test.TuneShooterVelocity.vel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Test.TuneShooterVelocity;

public class ShooterIO {
    private final DcMotorEx yawMotor;
    private final Servo pitchServo;
    public double Vel;
    private final DcMotorEx launcherMotorTop;
    private final DcMotorEx launcherMotorBottom;

    private final HardwareMap hardwareMap;
    public static final PIDFController shooterController = new PIDFController(TuneShooterVelocity.shooterCoeffs);
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            0.015, 0.9, 0.0003, 0
    );
    public ShooterIO(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;

            yawMotor = hardwareMap.get(DcMotorEx.class, "shooterYaw");
            pitchServo = hardwareMap.get(Servo.class, "shooterPitch");

            launcherMotorTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
            launcherMotorBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");

            yawMotor.setPositionPIDFCoefficients(100);
            yawMotor.setTargetPositionTolerance(10);

            launcherMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setYaw(double angle) {
        if (!(Math.abs(angle) > Math.PI/2)) {
            double motorTicks = 28.0;        // ticks por vuelta del motor
            double ratio = 36.0 * 1.9;       // relación total (68.4)
            double ticksPerRevFinal = motorTicks * ratio; // 1915.2

            int target = (int) Math.round((angle / (2 * Math.PI)) * ticksPerRevFinal);

            yawMotor.setTargetPosition(target);
            yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            yawMotor.setPower(1);
            return;
        }

        stopYaw();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public double getYawVel() {
        return yawMotor.getVelocity();
    }

    public void stopYaw() {
        yawMotor.setVelocity(0);
    }

    public void setPitch(double pos) {

        pitchServo.setPosition(pos);

        //pitchServo.setPosition(angle);
    }

    public void setVel() {
        shooterController.setCoefficients(shooterCoeffs);

        shooterController.setTolerance(20);

        double currentVelocity = launcherMotorTop.getVelocity();

        vel = (kA * vel) + shooterController.calculate(currentVelocity);

        vel = Math.max(-1, Math.min(vel, 1));
        if(Math.abs(shooterController.getSetPoint()) > 100){
            launcherMotorTop.setPower(vel);
            launcherMotorBottom.setPower(vel);
        } else{
            launcherMotorTop.setPower(0);
            launcherMotorBottom.setPower(0);
        }



    }

    public void setPoint(double setPoint){
        shooterController.setSetPoint(setPoint);
    }

    public void setPower(double power) {
        launcherMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotorTop.setPower(power);
        launcherMotorBottom.setPower(power);
;
    }

    public double getYaw() {
        return ((double) yawMotor.getCurrentPosition() / (28 * (135 / 70) * 36)) * (2 * Math.PI);
    }

    //ALEXIS
    // usen este mejor, no tiene sentido que devuelven un double si
    // hacen una división de enteros
    /*
    public double getYaw() {
        double ticksPerRevFinal = 28.0 * (135.0 / 70.0) * 36.0;
        return (yawMotor.getCurrentPosition() / ticksPerRevFinal) * (2.0 * Math.PI);
    }
    */

    public double getPitch() {
        return pitchServo.getPosition();
        //return pitchServo.getPosition();
    }

    public void setYawPower(double power) {;
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setPower(power);
    }

    public double getVel() {
        return launcherMotorTop.getVelocity();
    }
}
