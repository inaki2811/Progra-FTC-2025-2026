// IndexSubsystem.java
package org.firstinspires.ftc.teamcode.Inaki.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Inaki.core.Subsystem;

@Config
public class Index implements Subsystem {
    private DcMotorEx index;
    private ColorRangeSensor sensor; // opcional
    public double distance = 7;




    private final ElapsedTime timer = new ElapsedTime();
    private boolean scheduledAdvance = false;
    private double scheduledMs = 0;


    @Override
    public void init(HardwareMap hwMap) {
        index = hwMap.get(DcMotorEx.class, "index");
        index.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sensor = hwMap.get(ColorRangeSensor.class, "indexSensor");
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double RPM) {

        index.setPower(RPM);
    }

    public void stopIndex() {
        index.setVelocity(0);
        index.setPower(0);
    }

        public boolean isBallDetected() {
            double sensorDistance = sensor.getDistance(DistanceUnit.CM);

            return sensorDistance < distance;
        }


    public void scheduleAdvanceForMs(long ms, double ticksPerSec) {
        scheduledAdvance = true;
        scheduledMs = ms;
        timer.reset();
        index.setVelocity(ticksPerSec);
    }

    public boolean isAdvancing() { return scheduledAdvance; }

    @Override
    public void update() {
        if (scheduledAdvance && timer.milliseconds() >= scheduledMs) {
            stopIndex();
            scheduledAdvance = false;
        }
        if (isBallDetected()) stopIndex();
    }

    @Override public void stop() { stopIndex(); }
}