package org.firstinspires.ftc.teamcode.relicbeta.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a Robot.
 *
 */
public class HardwareQ4Way
{

    /* Public OpMode members. */
    public DcMotor leftmotor = null;
    public DcMotor rightmotor = null;
    public DcMotor frontmotor = null;
    public DcMotor backmotor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareQ4Way(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and Initialize Motors
        leftmotor  = initMotor("leftmotor", true);
        rightmotor = initMotor("rightmotor", false);
        backmotor   = initMotor("backmotor", true);
        frontmotor  = initMotor("frontmotor", false);

        // Define and initialize ALL installed servos.
       // rightButtonPusher = initServo("right_button_push", 0.0, false);
       // leftButtonPusher  = initServo("left_button_push", 1.0, false);
        //dropper           = initServo("dropper", 0.0, false);
        //cattleGuard       = initServo("cattleguard", 0.0, true);
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
}

