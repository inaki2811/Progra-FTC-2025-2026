package org.firstinspires.ftc.teamcode.Inaki.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="TestColorSensor", group="Test")
public class TestColorSensor extends OpMode {

    private NormalizedColorSensor colorSensor;

    @Override
    public void init() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

    }

    @Override
    public void loop() {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        colorSensor.setGain(60);

        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.update();
    }
}