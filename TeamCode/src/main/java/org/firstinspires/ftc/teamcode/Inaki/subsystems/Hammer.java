// HammerSubsystem.java
package org.firstinspires.ftc.teamcode.Inaki.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Inaki.core.Subsystem;

public class Hammer implements Subsystem {
    private Servo hammer;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean scheduledRetract = false;
    private double retractMs = 0;

    @Override
    public void init(HardwareMap hwMap) {
        hammer = hwMap.get(Servo.class, "hammer");
        retract();
    }

    public void push() { hammer.setPosition(1.0); }
    public void retract() { hammer.setPosition(0.0); }

    public void scheduleRetractForMs(long ms) {
        scheduledRetract = true;
        retractMs = ms;
        timer.reset();
        push();
    }

    public boolean isScheduled() { return scheduledRetract; }

    @Override
    public void update() {
        if (scheduledRetract && timer.milliseconds() >= retractMs) {
            retract();
            scheduledRetract = false;
        }
    }

    @Override public void stop() { retract(); }
}