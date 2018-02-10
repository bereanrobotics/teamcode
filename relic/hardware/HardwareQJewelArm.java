package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQJewelArm {

    /* Public OpMode members. */
    public Servo jewelArmLift = null;
    public ColorSensor sensorColor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareQJewelArm(){}

    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        //srv.setPosition(pos);
        return srv;
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and initialize ALL installed servos.
        jewelArmLift = initServo(HardwareQConstants.JEWEL_ARM_SERVONAME, HardwareQConstants.MID_SERVO, false);
        sensorColor = hwMap.get(ColorSensor.class, HardwareQConstants.COLOR_SENSORNAME);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
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
