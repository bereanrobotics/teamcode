package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/*
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a Robot.
 * It contains all the common hardware functions that don't change
 * across robots or seasons.
 *
 * This class should be subclassed to create specific robot hardware
 * either one per season or one per squad per season.
 *
 */
public class BereanHardware {

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry = null;
    protected ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public BereanHardware() {

    }

    /* handle standard motor initialization */
    protected DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }


    /* handle encoded motor initialization */
    protected DcMotor initMotorEncoded(String name, boolean reverse) {
        DcMotor motor = initMotor(name,reverse);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return motor;
    }


    /* handle standard servo initialization */
    protected Servo initServo(String name, double pos,  boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }

    /* Initialize standard Hardware interfaces
    *
    *  Subclasses should call this init before executing their own init functions
    *
    * */
    public void init(HardwareMap ahwMap, Telemetry aTelemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions
        telemetry = aTelemetry;
    }


}

