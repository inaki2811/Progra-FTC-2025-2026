package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Index;
import org.firstinspires.ftc.teamcode.Inaki.subsystems.Intake.IntakeIO;

@Config
@TeleOp(name="TestIntakeIndexColor", group="Test")
public class TestIntakeIndexColor extends OpMode {
    private IntakeIO intakeIO;
    private Index index;
    private boolean indexStopped = false;

    @Override
    public void init() {
        intakeIO = new IntakeIO();
        intakeIO.init(hardwareMap);

        index = new Index();
        index.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (!indexStopped) {
            intakeIO.setVelocity(-400);
            index.setVelocity(-1);
            boolean color = index.isBallDetected();
            telemetry.addData("Color detectado", color);

            if (color) {
                index.setVelocity(0);
                indexStopped = true;

            }
        } else {
            intakeIO.setVelocity(-400);
            index.setVelocity(0);

            if (gamepad1.a) {
                indexStopped = false;
            }
        }
        if (gamepad1.x){
            index.setVelocity(0);
            indexStopped = true;
            intakeIO.setVelocity(0);
            telemetry.addData("Index detenido", indexStopped);
            telemetry.addData("IntakeIO detenido", indexStopped);
            telemetry.update();
        }

        telemetry.addData("Index detenido", indexStopped);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeIO.setVelocity(0);
        index.setVelocity(0);
    }
}