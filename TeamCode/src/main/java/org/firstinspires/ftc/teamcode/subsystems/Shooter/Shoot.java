package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shoot {

    private final ShooterIO io;

    public Shoot(HardwareMap hardwareMap ) {io = new ShooterIO(hardwareMap);}

    public void setVel (double velocity){
        io.setPoint(velocity);
        io.setVel();

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
        return io.atVelocity();
    }




}
