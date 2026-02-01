package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class Shoot {

    private final ShooterIO shooterIO;
    private final IntakeIO intakeIO;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;
    private final Telemetry telemetry;
    private ElapsedTime elapsedTime;
    private ElapsedTime finishTemp;
    private boolean initialized  = false;
    private double distance;
    private double pitch;
    private double yaw;
    private final Supplier<AprilTagDetection> tagDetection;
    private double PowerShooter;
    private double velOffset;



    public Shoot(HardwareMap hardwareMap , Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Supplier<AprilTagDetection> tagDetection, double velOffset, Telemetry telemetry) {
        this.shooterIO = new ShooterIO(hardwareMap);
        this.intakeIO = new IntakeIO(shooterIO.getHardwareMap());
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;

        this.telemetry = telemetry;
        this.tagDetection = tagDetection;

        this.velOffset = velOffset;

        finishTemp = new ElapsedTime();
    }

    public void setVel (double velocity){
        shooterIO.setPoint(velocity);
        shooterIO.setVel();

    }

    public double getDistance(){
        distance = Math.hypot(distanceWithTargetX.get(), distanceWithTargetY.get());
        return distance;
    }

    public Action intake(){
        return packet -> {
        setVel(300);
        return false;
        };
    }

    public Action stop(){
        return packet -> {
        setVel(0);
        return false;
        };
    }

    public boolean atVelocity() {
        return shooterIO.atVelocity();
    }

    public Action prepareVelocity(){
        return telemetryPacket -> {
            if (!initialized) {
                elapsedTime = new ElapsedTime();
                initialized = true;
                //ALEXIS
                // finishedTemp debe de ser reseteado
                // finishTemp = new ElapsedTime(); o un reset

            }

            distance = getDistance();

            if (distance < 59.0551) {
                PowerShooter = 600;
            } else {
                PowerShooter = 700;

            }
            setVel(PowerShooter);
            return !isFinished();




        };


    }

    public boolean isFinished() {
        return Math.abs(shooterIO.getVelocity() - PowerShooter) < 50;
        //return false;
    }

    public Action trackingYaw(){
        return packet -> {

            distance = getDistance();

            yaw = Math.atan2(distanceWithTargetY.get(),
                    distanceWithTargetX.get())
                    - botYaw.get();
            shooterIO.setYaw(yaw);
            return false;
        };
    }

    public Action trackingHood(){
        return packet -> {

            distance = getDistance();


            if (distance < 59.0551) {
                pitch = -0.0000231316 * Math.pow(distance, 2)
                        - 0.0111474 * distance
                        + 1.28786;


            } else {
                pitch = -0.0000818672 * Math.pow(distance, 2)
                        + 0.00602864 * distance
                        + 0.972094;


            }

            shooterIO.setHood(pitch);
            return false;
        };


    }

}
