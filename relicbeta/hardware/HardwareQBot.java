package org.firstinspires.ftc.teamcode.relicbeta.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a Robot.
 *
 */
public class HardwareQBot
{
    public static final double MID_SERVO       =  0.5 ;
    public static final int LEFT = 1;
    public static final int RIGHT = 2;
    public static final int FORWARD_LEFT = 3;
    public static final int FORWARD_RIGHT = 4;
    public static final int BACKWARD_LEFT = 5;
    public static final int BACKWARD_RIGHT = 6;
    public static final int FORWARD = 7;
    public static final int BACKWARD = 8;

    /* Public OpMode members. */
    public DcMotor leftmotor = null;
    public DcMotor rightmotor = null;
    public DcMotor frontmotor = null;
    public DcMotor backmotor = null;
    public DcMotor motor180 = null;
    public DcMotor motorRack = null;
    public Servo glyphLeft = null;
    public Servo glyphRight = null;

    /* Autonomous timing */
    public double turnRotate = .5;
    public double driveOff;
    public double turnParallel = 1.576;
    public double driveOut = .09;

    public int motor180MaxPosition = 3000;
    public double motor180Power = 0.25;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareQBot(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }


    /* handle standard motor initialization */
    private DcMotor initMotorEncoded(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        //motorRack = initMotor("motorrack", false);
        motor180 = initMotorEncoded("motor180", false);
        // Define and initialize ALL installed servos.
        glyphLeft = initServo("glyphleft", MID_SERVO, false);
        glyphRight = initServo("glyphright", MID_SERVO, false);
    }

    // moves motor180 to a given position in its range
    // should convert this to a generic encoder move routine
    public void motor180SetPosition( int armPos ) {
        armPos = Range.clip( armPos, 0, motor180MaxPosition );
        if ( armPos != motor180.getTargetPosition() ) {
            motor180.setTargetPosition( armPos );
            motor180.setPower( motor180Power );
        } else if ( !motor180.isBusy() ) {
            motor180.setPower( 0 );
        }
    }

}

