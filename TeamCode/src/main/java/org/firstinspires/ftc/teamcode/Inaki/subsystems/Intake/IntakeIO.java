package org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeIO {
    private final DcMotorEx intake, index;
    private ColorRangeSensor sensor;


    public IntakeIO(HardwareMap hwMap) {
        sensor = hwMap.get(ColorRangeSensor.class, "indexSensor");

        intake = hwMap.get(DcMotorEx.class, "intakeMotor");
        index = hwMap.get(DcMotorEx.class, "indexMotor");

    }

    /// INTAKE ///

    public void setVelIntake(double vel) {
        intake.setVelocity(-vel);
    }

    public void setPwrIntake(double power) {
        intake.setPower(power);
    }

    public void setPositionIntake(double position){
        intake.setTargetPosition((int)Math.round(position));
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(1);
    }

    public double getVelIntake() {
        return intake.getVelocity();
    }


    /// INDEX ///


    public void setVelIndex(double vel) {
        index.setVelocity(-vel);
    }

    public void setPwrIndex(double power) {
        index.setPower(power);
    }

    public void setPositionIndex(double position){
        index.setTargetPosition((int)Math.round(position));
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    public double getVelIndex() {
        return index.getVelocity();
    }


    /// SENSOR ///


}
