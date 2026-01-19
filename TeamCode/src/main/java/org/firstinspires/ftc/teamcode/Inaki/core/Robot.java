package org.firstinspires.ftc.teamcode.Inaki.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private final List<Subsystem> subsystems = new ArrayList<>();
    private final Telemetry telemetry;

    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void initAll(HardwareMap hwMap) {
        for (Subsystem s : subsystems) {
            s.init(hwMap);
        }
    }

    public void updateAll() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void stopAll() {
        for (Subsystem s : subsystems) {
            s.stop();
        }
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
