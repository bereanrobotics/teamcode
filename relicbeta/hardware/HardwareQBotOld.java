package org.firstinspires.ftc.teamcode.relicbeta.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a AimBot.
 *
 */
public class HardwareQBotOld
{

    /* Public OpMode members. */
    public DeviceInterfaceModule cdi = null; // core device interface
    public DcMotor spinner = null;
    public DcMotor catapultMotor = null;
    public DcMotor front_right = null;
    public DcMotor front_left = null;
    public DcMotor back_right = null;
    public DcMotor back_left = null;
    public DcMotor forkLiftMotor = null;
    public DcMotor forkLiftMotor2 = null;
    public Servo Qermy = null;
    public Servo pusherLeft = null;
    public Servo pusherRight = null;
    public Servo forkLiftDropper = null;
    public DigitalChannel launchButton = null;
    public ModernRoboticsI2cGyro gyro = null;
    public LightSensor lightSensor = null;
    public LightSensor lightSensor2 = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public OpticalDistanceSensor ods = null;
    public UltrasonicSensor rangeSensor2 = null;

    public DigitalChannel rLED;
    public DigitalChannel gLED;
    public DigitalChannel bLED;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0; // 1120 or 28? eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // magic low level access to the MR color sensor as an i2c device
    private I2cDevice colorC;
    private I2cDeviceSynch colorCreader;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareQBotOld(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        RobotLog.d("HardwareQbot init motor: " + name);
        return motor;
    }
    private DcMotor initMotorWithEncoder(String name, boolean reverse, boolean zeroBreak) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (zeroBreak) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        RobotLog.d("HardwareQbot init motor with encoder: " + name);
        return motor;
    }

    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        //srv.setPosition(pos);
        RobotLog.d("HardwareQbot init servo: " + name);
        return srv;
    }

    // we have to read directly from the I2c port since MR doesn't let us read what we need.
    private void initColorSensor(String name)  {
        colorC = hwMap.i2cDevice.get(name);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 1);  // put sensor in Passive mode (0 for active)
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.d("HardwareQbot is initializing");
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and Initialize Motors

        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        rangeSensor2 = hwMap.get(UltrasonicSensor.class,"sensor_range2");
        catapultMotor = initMotorWithEncoder("meme", true, false);
        front_right = initMotorWithEncoder("front_right", false, false);
        front_left = initMotorWithEncoder("front_left", true, false);
        back_right = initMotorWithEncoder("back_right", false, false);
        back_left = initMotorWithEncoder("back_left", true, false);
        spinner = initMotor("spinner", false);
        forkLiftMotor = initMotor("fLMotor", true);
        forkLiftMotor2 = initMotor("fLMotor2", true);
        Qermy = initServo("qermy", 0.49019608, false);
        //pusherLeft = initServo("pusher1", 0, false);
        //pusherRight = initServo("pusher2", 0, true);
        forkLiftDropper = initServo("fLDropper", 0, false);
        // save a reference to the core device interface to set LED lights
        cdi = hwMap.deviceInterfaceModule.get("cdi");
        initColorSensor("cs");
        //gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        launchButton = hwMap.digitalChannel.get("launchButton");
        launchButton.setMode(DigitalChannelController.Mode.INPUT);
        // get a reference to our Light Sensor object.
        lightSensor = hwMap.opticalDistanceSensor.get("light");  // Alternative MR ODS sensor.
        lightSensor.enableLed(true);
        lightSensor2 = hwMap.opticalDistanceSensor.get("light2");
        lightSensor2.enableLed(true);

        rLED = hwMap.digitalChannel.get("r");
        rLED.setMode(DigitalChannelController.Mode.OUTPUT);
        gLED = hwMap.digitalChannel.get("g");
        gLED.setMode(DigitalChannelController.Mode.OUTPUT);
        bLED = hwMap.digitalChannel.get("b");
        bLED.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    // Power the left and right wheels as needed
    public void drive(double left, double right) {
        front_left.setPower(left);
        back_left.setPower(left);
        front_right.setPower(right);
        back_right.setPower(right);
    }
    public void driveSideways(double left, double right)
    {
        //90 degree shift
        //front_left is logically back_left
        //front_right is logically now front_left
        //back_left is now back_right
        //back_right is now front_right
        front_left.setPower(right);
        back_left.setPower(-left);
        front_right.setPower(-right);
        back_right.setPower(left);
    }

    public void setMode(DcMotor.RunMode mode) {
        front_left.setMode(mode);
        back_left.setMode(mode);
        front_right.setMode(mode);
        back_right.setMode(mode);
    }

    public void setTargetPosition(double left, double right) {
        front_left.setTargetPosition(front_left.getCurrentPosition() + (int) (left * COUNTS_PER_INCH));
        back_left.setTargetPosition(back_left.getCurrentPosition() + (int) (left * COUNTS_PER_INCH));
        front_right.setTargetPosition(front_right.getCurrentPosition() + (int) (right * COUNTS_PER_INCH));
        back_right.setTargetPosition(back_right.getCurrentPosition() + (int) (right * COUNTS_PER_INCH));
    }

    public boolean isDriving() {
        /* telemetry.addData("Driving:",  "Running at %7d :%7d %7d :%7d",
                front_left.getCurrentPosition(),
                back_left.getCurrentPosition(),
                front_right.getCurrentPosition(),
                back_right.getCurrentPosition());
*/
        return (front_left.isBusy() && back_left.isBusy() && front_right.isBusy() && back_right.isBusy());
    }

    public int getColorNumber() {
        byte[] colorCcache;
        colorCcache = colorCreader.read(0x04, 1);
        return(colorCcache[0] & 0xFF);
    }

    public void redLED(boolean state) {
        cdi.setLED(1, state);
        rLED.setState(state);
    }

    public void blueLED(boolean state) {
        cdi.setLED(0, state);
        bLED.setState(state);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mdSec.
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

