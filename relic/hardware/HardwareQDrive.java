package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQDrive {

    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor  = null;
    public DcMotor leftfrontmotor  = null;
    public DcMotor leftbackmotor   = null;

    HardwareMap hwMap =  null;
    private ElapsedTime runtime  = new ElapsedTime();

    private Telemetry telemetry;
    public boolean isAuto;

    public HardwareQDrive(){}

    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void init(HardwareMap ahwMap, Telemetry telemetryPassed, boolean isAutonomous) {

        hwMap = ahwMap;
        telemetry = telemetryPassed;
        isAuto = isAutonomous;

        rightfrontmotor = initMotor(HardwareQConstants.RIGHT_FRONT_MOTORNAME, false);
        rightbackmotor  = initMotor(HardwareQConstants.RIGHT_BACK_MOTORNAME, false);
        leftfrontmotor  = initMotor(HardwareQConstants.LEFT_FRONT_MOTORNAME, true);
        leftbackmotor   = initMotor(HardwareQConstants.LEFT_BACK_MOTORNAME, true );

    }

    public void strafe (int direction, double straightTime, double speed)
    {
            double startTime = runtime.seconds();



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


        while ((runtime.seconds()-startTime) < straightTime) //test this too
        {   this.telemetry.update();
        }
        if (isAuto == true) {

            stopMoving();
        }
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

    private void stopMoving() {
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
    }
}