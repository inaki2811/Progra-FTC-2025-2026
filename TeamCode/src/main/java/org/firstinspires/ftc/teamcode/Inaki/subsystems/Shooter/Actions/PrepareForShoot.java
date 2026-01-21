package org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.Actions;

import static android.icu.lang.UProperty.MATH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class PrepareForShoot implements Action {
    private final ShooterIO io;
    private final IntakeIO intakeIO;
    private MecanumDrive drive;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;

    private final double targetHeight = 0.5;
    private double accel = 9.81;
    private final Telemetry telemetry;

    private ElapsedTime elapsedTime;
    private ElapsedTime finishTemp;
    private boolean initialized  = false;

    private double pitch;
    private double yaw;
    private double vel;
    private double distance;

    private final Supplier<AprilTagDetection> tagDetection;

    private double velOffset;
    private double lastError = 0;
    private double integral = 0.0;

    private final double kP = 0.00005;
    private final double kI = 0.000;
    private final double kD = 0.002;
    private double PowerShooter;



    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Supplier<AprilTagDetection> tagDetection, double velOffset, Telemetry telemetry) {
        this.io = io;
        this.intakeIO = new IntakeIO(io.getHardwareMap());
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;
        this.drive = new MecanumDrive(io.getHardwareMap(),new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble()));

        this.telemetry = telemetry;
        this.tagDetection = tagDetection;

        this.velOffset = velOffset;

        finishTemp = new ElapsedTime();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double yawPower = 0;

        if (!initialized) {
            elapsedTime = new ElapsedTime();
            initialized = true;
            //ALEXIS
            // finishedTemp debe de ser reseteado
            // finishTemp = new ElapsedTime(); o un reset

        }
        distance = Math.hypot(distanceWithTargetX.get(), distanceWithTargetY.get());
        yaw = Math.atan2(distanceWithTargetY.get(),
                distanceWithTargetX.get())
                - botYaw.get();

        /*if (tagDetection.get() == null) {
            // ---- CÁLCULO DEL YAW---
             yaw = Math.atan2(distanceWithTargetY.get() * 0.0254,
                    distanceWithTargetX.get() * 0.0254)
                    - botYaw.get();
            distance = Math.sqrt(distanceWithTargetY.get() * 0.0254 * distanceWithTargetY.get() * 0.0254 +
                    distanceWithTargetX.get() * 0.0254 * distanceWithTargetX.get() * 0.0254);

            io.setYaw(Math.atan2(distanceWithTargetY.get(), distanceWithTargetX.get()));

            //ALEXIS
            // deberían de setear el power to 0 si no detecta el tag
        }/* else {
            AprilTagPoseFtc pose = tagDetection.get().ftcPose;

            double dt = elapsedTime.seconds();
            elapsedTime.reset();

            if (dt < 0.001) dt = 0.001;
            if (dt > 1.0) dt = 1.0;

            double imageCenterX = 640 / 2.0;
            double tagX = tagDetection.get().center.x;

            double error = -(tagX - imageCenterX);

            //ALEXIS
            // se tiene que agregar el error a la integral
            // integral += error * dt;
            integral = Math.max(-50, Math.min(50, integral));

            double derivative = (error - lastError) / dt;

            double pidOutput =
                    (kP * error) + (kI * integral) + (kD * derivative);

            if (Math.abs(error) < 15) {
                pidOutput = 0;
                integral = 0;
            }

            yawPower = Math.max(
                    -1,
                    Math.min(1, pidOutput)
            );

            if (Double.isInfinite(yawPower)) {
                yawPower = 0;
                integral = 0;
                lastError = 0;
            }

            io.setYawPower(yawPower);

            lastError = error;

            distance = Math.sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z) * 0.0254;

        }*/

        if (distance < 59.0551) {
            pitch = -0.0000231316 * Math.pow(distance, 2)
                    - 0.0111474 * distance
                    + 1.28786;
            vel = 300;
        } else {
            pitch = -0.0000818672 * Math.pow(distance, 2)
                    + 0.00602864 * distance
                    + 0.972094;
            vel = 400;
        }


        io.setHood(pitch);


        if (Math.abs(yaw) < 60 ){

            io.setYaw(yaw);

        }else{

            io.setYaw(0);/*
            if(Math.abs(yaw) > 60){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),yaw));

                drive.s

            }

            return false;*/

        }

        //ALEXIS
        // usen veloffset
        // vel = -1400 + velOffset;

        //ALEXIS
        // yaw y vel no los inicializan con ningun valor mas que hasta arriba
        // deben de darles un valor, sino no va a hacer nada las siguientes 2 lóneas
        // yaw = io.getYaw();
        // io.setVel(vel);
        io.setVel();
        telemetry.addData("desiredShooterPitch", pitch);
        telemetry.addData("errorShooter", lastError);
        telemetry.addData("distance", distance);
        telemetry.addData("power", yawPower);
        telemetry.addData("desiredShooterYaw", yaw);
        telemetry.addData("desiredShooterVel", PowerShooter);
        telemetry.addData("vel", io.getVelocity());
        telemetry.addData("desiredShooterX", distanceWithTargetX.get());
        telemetry.addData("desiredShooterY", distanceWithTargetY.get());
        // ALEXIS
        // chequen en agregar esto
        // telemetry.addData("yawError", lastError);
        // telemetry.addData("currentShooterYaw", yaw);
        return !isFinished();
    }

    public boolean isFinished() {
        return Math.abs(io.getVelocity() - PowerShooter) < 50;
        //return false;
    }
}
