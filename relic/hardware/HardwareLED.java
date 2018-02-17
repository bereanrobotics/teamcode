package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.BereanHardware;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareLED extends BereanHardware {

    public DigitalChannel rLED;
    public DigitalChannel gLED;
    public DigitalChannel bLED;

    /* Constructor */
    public HardwareLED(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetryPassed) {
        // Save reference to Hardware map
        super.init(ahwMap, telemetryPassed);

 /* Public OpMode members. */
        rLED = hwMap.digitalChannel.get("r");
        rLED.setMode(DigitalChannelController.Mode.OUTPUT);
        gLED = hwMap.digitalChannel.get("g");
        gLED.setMode(DigitalChannelController.Mode.OUTPUT);
        bLED = hwMap.digitalChannel.get("b");
        bLED.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    public void redLED(boolean state) {
        rLED.setState(state);
    }

    public void blueLED(boolean state) {
        bLED.setState(state);
    }
}

