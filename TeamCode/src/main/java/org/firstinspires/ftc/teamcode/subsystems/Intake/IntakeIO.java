package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeIO {
    private ColorRangeSensor sensor;
    private final DcMotorEx intake, index;


    public IntakeIO(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorRangeSensor.class, "indexSensor");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        index = hardwareMap.get(DcMotorEx.class, "index");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
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

    public double getDistanceSensor() { return sensor.getDistance(DistanceUnit.CM);}


}
