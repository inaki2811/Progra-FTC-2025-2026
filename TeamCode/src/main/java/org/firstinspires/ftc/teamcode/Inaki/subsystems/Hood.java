package org.firstinspires.ftc.teamcode.Inaki.subsystems;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.Inaki.core.Subsystem;

public class Hood implements Subsystem {
    private Servo hood;
    private double targetPos = 0;
    private double tolerance = 0.02;

    @Override
    public void init(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");
    }

    public void setTarget(double pos) {
        targetPos = pos;
        hood.setPosition(pos);
    }

    public boolean atPosition() {
        double current = hood.getPosition();
        return Math.abs(current - targetPos) <= tolerance;
    }

    @Override public void update() {}
    @Override public void stop() {}
}