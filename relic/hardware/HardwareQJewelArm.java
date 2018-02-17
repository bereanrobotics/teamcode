package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.BereanHardware;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQJewelArm extends BereanHardware {

    /* Public OpMode members. */
    public static final double JEWEL_ARM_RESTING_POSITION = 0.0;
    public Servo jewelArmLift = null;
    public ColorSensor sensorColor = null;

    /* Constructor */
    public HardwareQJewelArm(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetryPassed) {
        // Save reference to Hardware map
        super.init(ahwMap, telemetryPassed);

        // Define and initialize ALL installed servos.
        jewelArmLift = initServo(HardwareQConstants.JEWEL_ARM_SERVONAME, HardwareQConstants.MID_SERVO, false);
        sensorColor = hwMap.get(ColorSensor.class, HardwareQConstants.COLOR_SENSORNAME);
    }


    public void deploy()
    {
        jewelArmLift.setPosition(1);
    }

    public void retract()
    {
        jewelArmLift.setPosition(0);
    }

    public int getDirectionToHitJewel(int teamColor)
    {
        //get sensor color
        // if sensor color == team color
        return HardwareQConstants.LEFT;
        // else return right;
    }

}
