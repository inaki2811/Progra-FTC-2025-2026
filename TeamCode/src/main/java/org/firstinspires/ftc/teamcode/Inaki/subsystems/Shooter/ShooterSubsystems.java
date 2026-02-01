package org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.Actions.PrepareForShoot;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Shooter.Actions.SetVel;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class ShooterSubsystems {
    private final ShooterIO io;
    private final HardwareMap hardwareMap;

    public ShooterSubsystems(HardwareMap hardwareMap) {
        io = new ShooterIO(hardwareMap);
        this.hardwareMap = hardwareMap;
    }

    public Action prepareForShoot(Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Supplier<AprilTagDetection> tagDetection, double velOffset, Telemetry telemetry) {
        return new PrepareForShoot(io, distanceWithTargetX, distanceWithTargetY, botYaw, tagDetection, velOffset, telemetry);
    }

    public Action intake() {return new SetVel(io , -150);}

    public Action stop() {return new SetVel(io, 0);}

    public ShooterIO getIO() {
        return io;
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addData("VelShooter", io.getVelocity());
        telemetry.addData("shooterYaw", io.getYaw());
        telemetry.addData("shooterPitch", io.getHood());
        io.setVel();


    }

}