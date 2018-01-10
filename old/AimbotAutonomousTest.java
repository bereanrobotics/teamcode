/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file provides  Telop driving for Aimbot.
 */

@Autonomous(name="Aimbot - Autonomous Test", group="Aimbot")
@Disabled
public class AimbotAutonomousTest extends LinearOpMode {



    /* Declare OpMode members. */

    HardwareAimbot robot = new HardwareAimbot(); // use the class created to define a Aimbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 28.0; // 1120 or 28? eg: AndyMark NeverRest40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = .2;
    static final double FORWARD_SPEED = 0.5;
    static final double TURN_SPEED = 0.2;


    public void DriveForward(int time) throws InterruptedException {
        robot.frontLeftMotor.setPower(FORWARD_SPEED);
        robot.backLeftMotor.setPower(FORWARD_SPEED);
        robot.frontRightMotor.setPower(FORWARD_SPEED);
        robot.backRightMotor.setPower(FORWARD_SPEED);
        sleep(time);
    }

    public void DrivBackwards(int time) throws InterruptedException {
        robot.frontLeftMotor.setPower(-FORWARD_SPEED);
        robot.backLeftMotor.setPower(-FORWARD_SPEED);
        robot.frontRightMotor.setPower(-FORWARD_SPEED);
        robot.backRightMotor.setPower(-FORWARD_SPEED);
        sleep(time);


    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches1, double leftInches2, double rightInches1, double rightInches2,
                             double timeoutS) throws InterruptedException {
        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;

        DcMotor tLeft = robot.frontLeftMotor;
        DcMotor tRight = robot.frontRightMotor;
        DcMotor bLeft = robot.backLeftMotor;
        DcMotor bRight = robot.backRightMotor;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_tLeftTarget = tLeft.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
            new_tRightTarget = tRight.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
            new_bLeftTarget = bLeft.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);
            new_bRightTarget = bRight.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);
            tLeft.setTargetPosition(new_tLeftTarget);
            tRight.setTargetPosition(new_tRightTarget);
            bLeft.setTargetPosition(new_bLeftTarget);
            bRight.setTargetPosition(new_bRightTarget);

            // Turn On RUN_TO_POSITION
            tLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            tLeft.setPower(speed);
            tRight.setPower(speed);
            bLeft.setPower(speed);
            bRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (tLeft.isBusy() && tRight.isBusy() && bLeft.isBusy() && bRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", new_tLeftTarget, new_tRightTarget, new_bLeftTarget, new_bRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        tLeft.getCurrentPosition(),
                        tRight.getCurrentPosition(),
                        bLeft.getCurrentPosition(),
                        bRight.getCurrentPosition());
                telemetry.update();
                idle();
            }
        }

        // Stop all motion;
        tLeft.setPower(0);
        tRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

        // Turn off RUN_TO_POSITION
        tLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }


    @Override
    public void runOpMode() throws InterruptedException {


         /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(),
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(drive_Speed, tleft, tright, bleft, bright, timeout)
        robot.leftButtonPusher.setPosition(0);
        sleep(500);
        robot.rightButtonPusher.setPosition(1);
        sleep(500);
        encoderDrive(DRIVE_SPEED,24, 24, 24, 24, 10.0);  // S1: Forward 24 Inches
        //encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 1.0); // strafing left
        //encoderDrive(DRIVE_SPEED, 12, -12, -12, 12, 1.0); // strafing right
        encoderDrive(TURN_SPEED,   17.5, 17.5, -17.5, -17.5, 10.0);
        encoderDrive(DRIVE_SPEED, 26, 26, 26, 26, 10.0);
        encoderDrive(TURN_SPEED,-17.5,-17.5,17.5,17.5,10.0);
        encoderDrive(DRIVE_SPEED,28,28,28,28,10.0);
        encoderDrive(TURN_SPEED,17.5,17.5,-17.5,-17.5,10.0);
        encoderDrive(DRIVE_SPEED,31,31,31,31,10.0);
        sleep(500);
        robot.leftButtonPusher.setPosition(0);
        robot.leftButtonPusher.setPosition(1);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");

        telemetry.update();
    }
}


