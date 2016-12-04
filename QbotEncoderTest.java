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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Qbot: Encoder Test", group="Qbot")
@Disabled
public class QbotEncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    private HardwareQBot robot   = new HardwareQBot();   // Use a qbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28.0; // 1120 or 28? eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .2;
    static final double     TURN_SPEED              = .2;
    static final int        CATAPULT_LAUNCH_COUNT_SHORT   = 750;
    static final int        CATAPULT_LAUNCH_COUNT_LONG   = -750;

    private void setCatapultAndLaunch(boolean setServo, boolean longLaunch) throws InterruptedException
    {
        // first reset encoder so that the current position is the zero position
        robot.catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Catapult", "Reset");
        telemetry.update();
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < 1) && robot.catapultMotor.isBusy()) {
            telemetry.addData("Catapult", "Resetting");
            telemetry.update();
            idle();
        }

        if (longLaunch) {
            robot.catapultMotor.setTargetPosition(CATAPULT_LAUNCH_COUNT_LONG);
        } else {
            robot.catapultMotor.setTargetPosition(CATAPULT_LAUNCH_COUNT_SHORT);
        }
        robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.catapultMotor.setPower(0.5);

        runtime.reset();
        // Move catapult into loading position
        while (opModeIsActive() && (runtime.seconds() < 2) || robot.catapultMotor.isBusy()) {
            telemetry.addData("Catapult","Ready Position %d", robot.catapultMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.catapultMotor.setPower(0.0);
        idle();
        //sleep(2000);
        if (setServo)
        {
            robot.Qermy.setPosition(0.07843137);
            sleep(2000);
            robot.Qermy.setPosition(0.49019608);
            sleep(1000);
        }
        robot.catapultMotor.setTargetPosition(1120);
        robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Catapult", "Fired!");
        telemetry.update();
        runtime.reset();
        // keep looping while we are still active
        while (opModeIsActive() && (runtime.seconds() < 1) || robot.catapultMotor.isBusy()) {
            idle();
        }
        //sleep(2000);
        robot.catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Catapult", "Reset");
        telemetry.update();
        idle();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //


        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                robot.front_left.getCurrentPosition(),
                robot.front_right.getCurrentPosition(),
                robot.back_left.getCurrentPosition(),
                robot.back_right.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(drive_Speed, tleft, tright, bleft, bright, timeout)





        //Autonomous Red 1
        //Launch first ball
        setCatapultAndLaunch(false, false);
        //Knock ball into catapult and launch second ball
        setCatapultAndLaunch(true, false);






        //encoderDrive(DRIVE_SPEED,  36,  36, 36, 36, 10.0);  // S1: Forward 48 Inches

        //encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 1.0); // strafing left
        //encoderDrive(DRIVE_SPEED, 12, -12, -12, 12, 1.0); // strafing right
        //encoderDrive(TURN_SPEED,   12, 12, -12, -12, 1.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

        //encoderDrive(DRIVE_SPEED, -36, -36, -36, -36, 10.0);  // S3: Reverse 48 Inches with 4 Sec timeout


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
                             double leftInches1, double leftInches2, double rightInches1,double rightInches2,
                             double timeoutS) throws InterruptedException {
        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;

        DcMotor tLeft = robot.front_left;
        DcMotor tRight = robot.front_right;
        DcMotor bLeft = robot.back_left;
        DcMotor bRight = robot.back_right;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_tLeftTarget = tLeft.getCurrentPosition() + (int)(leftInches1 * COUNTS_PER_INCH);
            new_tRightTarget = tRight.getCurrentPosition() + (int)(rightInches1 * COUNTS_PER_INCH);
            new_bLeftTarget = bLeft.getCurrentPosition() + (int)(leftInches2 * COUNTS_PER_INCH);
            new_bRightTarget = bRight.getCurrentPosition() + (int)(rightInches2 * COUNTS_PER_INCH);
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
                    ( tLeft.isBusy() && tRight.isBusy()  && bLeft.isBusy() && bRight.isBusy())) {

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
}