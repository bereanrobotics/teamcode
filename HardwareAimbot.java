package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a AimBot.
 *
 */
public class HardwareAimbot
{

    /* Public OpMode members. */
    public DeviceInterfaceModule cdi = null; // core device interface
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    //public DcMotor dankMeme2016 = null;
    public Servo rightButtonPusher = null;
    public Servo leftButtonPusher = null;
    public Servo dropper = null;
    public Servo cattleGuard = null;

    // magic low level access to the MR color sensor as an i2c device
    private I2cDevice colorC;
    private I2cDeviceSynch colorCreader;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareAimbot(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private DcMotor initMotorWithEncoder(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
        return motor;
    }

    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }

    // we have to read directly from the I2c port since MR doesn't let us read what we need.
    private void initColorSensor()  {
        colorC = hwMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 1);  // put sensor in Passive mode (0 for active)
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and Initialize Motors


        frontLeftMotor  = initMotor("left_front", false);
        frontRightMotor = initMotor("right_front", true);
        backLeftMotor   = initMotor("left_back", false);
        backRightMotor  = initMotor("right_back", true);
        //catapultMotor = initMotor("meme", true);

        // Define and initialize ALL installed servos.
        rightButtonPusher = initServo("right_button_push", 0.0, false);
        leftButtonPusher  = initServo("left_button_push", 1.0, false);
        //dropper           = initServo("dropper", 0.0, false);
        //cattleGuard       = initServo("cattleguard", 0.0, true);

        // save a reference to the core device interface to set LED lights
        cdi = hwMap.deviceInterfaceModule.get("cdi");
        initColorSensor();

    }

    // Power the left and right wheels as needed
    public void drive(double left, double right) {
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
    }

    // shorthand for drive with all zeros
    public void park() {
        drive(0, 0);
    }

    public int getColorNumber() {
        byte[] colorCcache;
        colorCcache = colorCreader.read(0x04, 1);
        return(colorCcache[0] & 0xFF);
    }

    public void redLED(boolean state) {
        cdi.setLED(1, state);
    }

    public void blueLED(boolean state) {
        cdi.setLED(0, state);
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

