package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.BereanHardware;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQDrive extends BereanHardware {

    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor  = null;
    public DcMotor leftfrontmotor  = null;
    public DcMotor leftbackmotor   = null;

    //private Telemetry telemetry;
    public boolean isAuto;

    public HardwareQDrive(){}

    // moves encoded motor to a given position
    public void motorSetPosition( DcMotor motor, int position, double power ) {
         if ( position != motor.getTargetPosition() ) {
            motor.setTargetPosition( position );
            motor.setPower( power );
        } else if ( !motor.isBusy() ) {
            motor.setPower( 0 );
        }
    }

    public void init(HardwareMap ahwMap, Telemetry telemetryPassed ) {

        super.init(ahwMap,telemetryPassed);

        rightfrontmotor = initMotor(HardwareQConstants.RIGHT_FRONT_MOTORNAME, false);
        rightbackmotor  = initMotor(HardwareQConstants.RIGHT_BACK_MOTORNAME, false);
        leftfrontmotor  = initMotor(HardwareQConstants.LEFT_FRONT_MOTORNAME, true);
        leftbackmotor   = initMotor(HardwareQConstants.LEFT_BACK_MOTORNAME, true );
    }

    public void initEncoded(HardwareMap ahwMap, Telemetry telemetryPassed ) {

        super.init(ahwMap,telemetryPassed);

        rightfrontmotor = initMotorEncoded(HardwareQConstants.RIGHT_FRONT_MOTORNAME, false);
        rightbackmotor  = initMotorEncoded(HardwareQConstants.RIGHT_BACK_MOTORNAME, false);
        leftfrontmotor  = initMotorEncoded(HardwareQConstants.LEFT_FRONT_MOTORNAME, true);
        leftbackmotor   = initMotorEncoded(HardwareQConstants.LEFT_BACK_MOTORNAME, true );
    }

    public void strafe(int direction, double speed)
    {
        if (direction == HardwareQConstants.FORWARD)
        {
            this.telemetry.addData("Status", "driving forward");
            rightfrontmotor.setPower(speed);
            rightbackmotor.setPower(speed);
            leftfrontmotor.setPower(speed);
            leftbackmotor.setPower(speed);
        }

        if (direction == HardwareQConstants.BACKWARD)
        {
            this.telemetry.addData("Status", "driving back");

            rightfrontmotor.setPower(-speed);
            rightbackmotor.setPower(-speed);
            leftfrontmotor.setPower(-speed);
            leftbackmotor.setPower(-speed);
        }

        if (direction == HardwareQConstants.RIGHT)
        {
            this.telemetry.addData("Status", "strafing right");

            rightfrontmotor.setPower(-speed);
            rightbackmotor.setPower(speed);
            leftfrontmotor.setPower(speed);
            leftbackmotor.setPower(-speed);
        }

        if (direction == HardwareQConstants.LEFT)
        {
            this.telemetry.addData("Status", "driving left");

            rightfrontmotor.setPower(speed);
            rightbackmotor.setPower(-speed);
            leftfrontmotor.setPower(-speed);
            leftbackmotor.setPower(speed);
        }
    }

    public void strafeFor(int direction, double straightTime, double speed)
    {
        double startTime = runtime.seconds();

        strafe(direction,speed);

        while ((runtime.seconds()-startTime) < straightTime) //test this too
        {   this.telemetry.update();
        }

        stopMoving();

    }

    // this is not ideal since it uses direct encoder values rather than inches
    public void strafeTo(int direction, int position, double speed)
    {
        motorSetPosition(rightfrontmotor, position, speed);
        motorSetPosition(rightbackmotor, position, speed);
        motorSetPosition(leftfrontmotor, position, speed);
        motorSetPosition(leftbackmotor, position, speed);
    }

    public void rotateRobot(int direction, double rotateSeconds, double speed)
    {
        double startTime = runtime.seconds();

        if(direction == HardwareQConstants.LEFT)
        {
            rightfrontmotor.setPower(speed);
            rightbackmotor.setPower(speed);
            leftfrontmotor.setPower(-speed);
            leftbackmotor.setPower(-speed);
        } else
        {
            rightfrontmotor.setPower(-speed);
            rightbackmotor.setPower(-speed);
            leftfrontmotor.setPower(speed);
            leftbackmotor.setPower(speed);
        }

        while ((runtime.seconds()-startTime) < rotateSeconds) //test isOpModeActive()
        {
            this.telemetry.addData("Status", "rotating");
            this.telemetry.update();
        }

        stopMoving();
    }

    public void stopMoving() {
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
    }
}