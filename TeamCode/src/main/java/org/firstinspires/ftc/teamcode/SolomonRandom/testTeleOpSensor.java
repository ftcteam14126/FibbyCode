package org.firstinspires.ftc.teamcode.SolomonRandom;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="testTeleOpSensor", group="Sensor")
//@Disabled
public class testTeleOpSensor extends OpMode {
    // declaring variables
    private SensorMBRangeSensor rangeSensor;
    //private MB1242Ex rangeSensor;

    @Override
    public void init() {
        // define and initialize sensor (hardware mapping)
        rangeSensor = hardwareMap.get(SensorMBRangeSensor.class, "rangeSensor");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        telemetry.addData("rangeSensor", rangeSensor.getRange(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void stop() {}
}
