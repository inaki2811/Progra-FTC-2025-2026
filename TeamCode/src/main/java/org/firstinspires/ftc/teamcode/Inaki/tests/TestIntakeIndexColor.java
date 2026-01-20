package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Inaki.subsystems.Index;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;

@Config
@TeleOp(name="TestIntakeIndexColor", group="Test")
public class TestIntakeIndexColor extends OpMode {
    private IntakeIO intakeIO;

    private boolean indexStopped = false;


    @Override
    public void init() {
        intakeIO = new IntakeIO(hardwareMap);
    }

    @Override
    public void loop() {

        if (!indexStopped) {
            intakeIO.setVelIntake(100);
            intakeIO.setPwrIndex(-1);
            boolean color = intakeIO.isBallDetected();
            telemetry.addData("Color detectado", color);

            if (color) {
                intakeIO.setVelIndex(0);
                indexStopped = true;

            }
        } else {
            intakeIO.setVelIntake(100);
            intakeIO.setPwrIntake(0);

            if (gamepad1.a) {
                indexStopped = false;
            }
        }
        if (gamepad1.x){
            intakeIO.setVelIntake(0);
            indexStopped = true;
            intakeIO.setVelIndex(0);
            telemetry.addData("Index detenido", indexStopped);
            telemetry.addData("IntakeIO detenido", indexStopped);
            telemetry.update();
        }

        telemetry.addData("Index detenido", indexStopped);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeIO.setPwrIntake(0);
        intakeIO.setPwrIndex(0);
    }
}