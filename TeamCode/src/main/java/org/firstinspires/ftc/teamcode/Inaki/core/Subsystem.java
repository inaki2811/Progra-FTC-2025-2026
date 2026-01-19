package org.firstinspires.ftc.teamcode.Inaki.core;
import com.qualcomm.robotcore.hardware.HardwareMap;
public interface  Subsystem {
    void init(HardwareMap hwMap);
    void update();
    void stop();
    default String name() { return this.getClass().getSimpleName(); }
}