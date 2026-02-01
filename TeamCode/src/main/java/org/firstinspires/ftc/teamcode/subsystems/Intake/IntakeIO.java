package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeIO {
    private final DigitalChannel stage1LimitSwitch;
    private final DigitalChannel stage2LimitSwitch;
    private final DigitalChannel stage3LimitSwitch;
    private final DcMotorEx motor;
    private final Servo blockerL;
    private final Servo blockerR;

    public IntakeIO(HardwareMap hardwareMap) {
        stage1LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage1");
        stage2LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage2");
        stage3LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage3");

        blockerL = hardwareMap.get(Servo.class, "blockerL");
        blockerR = hardwareMap.get(Servo.class, "blockerR");

        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setVel(double vel) {
        motor.setVelocity(-vel);
    }

    public void setPwr(double power) {
        motor.setPower(power);
    }

    public void setBlockState(boolean block) {
        if (block) {
            blockerL.setPosition(0.5);
            blockerR.setPosition(0.5);

            return;
        }
        blockerL.setPosition(1);
        blockerR.setPosition(1);
    }

    public void setPosition(double position) {
        motor.setTargetPosition((int) Math.round(position));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public boolean onStage1() {
        return stage1LimitSwitch.getState();
    }

    public boolean onStage2() {
        return stage2LimitSwitch.getState();
    }

    public boolean onStage3() {
        return stage3LimitSwitch.getState();
    }

    public double getVel() {
        return motor.getVelocity();
    }

    public double getBlockerLPos() {
        return blockerL.getPosition();
    }

    public double getBlockerRPos() {
        return blockerR.getPosition();
    }
}