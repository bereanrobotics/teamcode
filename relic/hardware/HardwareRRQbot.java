package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a Robot.
 *
 */
public class HardwareRRQbot
{
    public static final double MID_SERVO     =  0.5 ;
    public static final int LEFT = 1;
    public static final int RIGHT = 2;
    public static final int FORWARD_LEFT = 3;
    public static final int FORWARD_RIGHT = 4;
    public static final int BACKWARD_LEFT = 5;
    public static final int BACKWARD_RIGHT = 6;
    public static final int FORWARD = 7;
    public static final int BACKWARD = 8;
    public static final double GLYPH_GRAB = 0.6;
    public static final double GLYPH_DROP = 0.0;

    /* Public OpMode members. */
    public DcMotor leftmotor = null;
    public DcMotor rightmotor = null;
    public DcMotor frontmotor = null;
    public DcMotor backmotor = null;
    public DcMotor motor180 = null;
    public DcMotor motorRack = null;
    public Servo glyphLeft = null;
    public Servo glyphRight = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public HardwareRRQbot(){

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
        motorRack = initMotor("motorrack", false);
        motor180 = initMotor("motor180", true);
        // Define and initialize ALL installed servos.
        glyphLeft = initServo("glyphleft", MID_SERVO, false);
        glyphRight = initServo("glyphright", MID_SERVO, false);
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

        long  remaining = periodMs - (long) runtime.milliseconds();

        // sleep for the remaining portion of the regular cycle runtime.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        runtime.reset();
    }

    /*******************************
     *
     * turnOffDriveTrain will shutdown all drive train motors.
     *
     * @param brake - boolean value.  If true, sets the power to zero and
     *              brakes the motor to hold it in place.  False sets the
     *              power mode to float enabling the wheels to rotate.
     */
    public void turnOffDriveTrain(boolean brake)
    {
        if(brake){
            leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else
        {
            leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);
        backmotor.setPower(0);
        frontmotor.setPower(0);
    }

    /**************************************************
     *
     * pauseDriveTrain will stop all robot driving for the runtime
     * of time, in seconds, specified by the pauseSeconds parameter.
     * Use this function in autonomous to tell the robot to stop moving around
     * for a specified period of time.  It's up to you to move the robot again
     * once the call returns.
     *
     * For drive-train components ONLY.
     *
     *
     * @param pauseSeconds
     ***************************************************/
    public void pauseDriveTrain(double pauseSeconds, Telemetry telemetry)
    {
        if (pauseSeconds < 0) return;

        runtime.reset(); // make sure the elapsed time is reset

        turnOffDriveTrain(false);

        while ( (runtime.seconds() < pauseSeconds))
        {
            telemetry.addData("Status", "pausing");
            telemetry.update();
        }
    }

    public void rotateRobot(int direction, double seconds, double speed, Telemetry telemetry)
    {
        if (direction != LEFT || direction != RIGHT) return;

        runtime.reset();

        if(direction == LEFT)
        {
            backmotor.setPower(-speed);
            frontmotor.setPower(speed);
            rightmotor.setPower(speed);
            leftmotor.setPower(-speed);
        } else
        {
            backmotor.setPower(speed);
            frontmotor.setPower(-speed);
            rightmotor.setPower(-speed);
            leftmotor.setPower(speed);
        }

        while (runtime.seconds() < seconds)
        {
            telemetry.addData("Status", "rotating");
            telemetry.update();
        }

        turnOffDriveTrain(false);
    }

    public void driveDiagonal (int direction, double seconds, double speed, Telemetry telemetry)
    {
        if (direction != FORWARD_LEFT||
                direction != FORWARD_RIGHT ||
                direction != BACKWARD_LEFT ||
                direction != BACKWARD_RIGHT ) return;

        runtime.reset();

        if (direction == FORWARD_LEFT)
        {
            backmotor.setPower(speed);
            frontmotor.setPower(speed);
            rightmotor.setPower(speed);
            leftmotor.setPower(speed);
        }

        if (direction == FORWARD_RIGHT)
        {
            backmotor.setPower(-speed);
            frontmotor.setPower(-speed);
            rightmotor.setPower(speed);
            leftmotor.setPower(speed);
        }

        if (direction == BACKWARD_LEFT)
        {
            backmotor.setPower(speed);
            frontmotor.setPower(speed);
            rightmotor.setPower(-speed);
            leftmotor.setPower(-speed);
        }

        if (direction == BACKWARD_RIGHT)
        {
            backmotor.setPower(-speed);
            frontmotor.setPower(-speed);
            rightmotor.setPower(-speed);
            leftmotor.setPower(-speed);
        }

        while (runtime.seconds() < seconds)
        {
            telemetry.addData("Status", "Robot is driving diagonally.");
            telemetry.update();
        }

        turnOffDriveTrain(false);
    }


    public void driveStrafe (int direction, double seconds, double speed, Telemetry telemetry)
    {
        if (direction != FORWARD ||
                direction != BACKWARD ||
                direction != LEFT ||
                direction != RIGHT ) return;

        runtime.reset();

        if (direction == FORWARD)
        {

            backmotor.setPower(0);
            frontmotor.setPower(0);
            rightmotor.setPower(speed);
            leftmotor.setPower(speed);
        }

        if (direction == BACKWARD)
        {
            backmotor.setPower(0);
            frontmotor.setPower(0);
            rightmotor.setPower(-speed);
            leftmotor.setPower(-speed);
        }

        if (direction == LEFT)
        {
            backmotor.setPower(-speed);
            frontmotor.setPower(-speed);
            rightmotor.setPower(0);
            leftmotor.setPower(0);
        }

        if (direction == RIGHT)
        {
            backmotor.setPower(speed);
            frontmotor.setPower(speed);
            rightmotor.setPower(0);
            leftmotor.setPower(0);
        }

        while (runtime.seconds() < seconds)
        {
            telemetry.addData("Status", "Robot is strafing.");
            telemetry.update();
        }

        turnOffDriveTrain(false);
    }

    public void glyph ( double position )
    {
        glyphLeft.setPosition(MID_SERVO + position);
        glyphRight.setPosition(MID_SERVO - position);
    }

    public void moveRack(double speed, double seconds , Telemetry telemetry)
    {
        runtime.reset();

        motorRack.setPower( speed);
        while (runtime.seconds() < seconds)
        {
            telemetry.addData("Status", "Moving the rack");
            telemetry.update();
        }
        motorRack.setPower( 0 );
    }

    public void moveArm(double speed, double seconds, Telemetry telemetry )
    {
        runtime.reset();
        motor180.setPower( speed);
        while (runtime.seconds() < seconds)
        {
            telemetry.addData("Status", "Moving the m180 Arm");
            telemetry.update();
        }
        motor180.setPower( 0 );
    }
}

