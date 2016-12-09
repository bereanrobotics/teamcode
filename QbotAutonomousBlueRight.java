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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Qbot: Blue/Right Autonomous", group="Qbot")
//@Disabled
public class QbotAutonomousBlueRight extends LinearOpMode {

    /* Declare OpMode members. */
    private HardwareQBot robot   = new HardwareQBot();   // Use a qbot's hardware

    static final double     COUNTS_PER_MOTOR_REV    = 28.0; // 1120 or 28? eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final int        FULL_ROTATION           = 1120;  //ticks in a full geared rotation
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final int     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;
    static final int        CATAPULT_LAUNCH_COUNT   = 435;
    static final int        SENSOR_RED     = 10;
    static final int        SENSOR_BLUE    = 3;

    static final double     PUSHER_DOWN    = 0.0;
    static final double     PUSHER_UP_RIGHT      = 0.439;
    static final double     PUSHER_UP_LEFT      = 0.568;
    double pusherRightPos = PUSHER_DOWN;
    double pusherLeftPos = PUSHER_DOWN;
    Boolean blueTeam = true; // flag to determine if we are the red or blue team.
    double ballLoaderOffset = 0.49019608;
    double ballLoaderDefaultPos = 0.49019608;
    double ballLoaderTargetPos = 0.07843137;

    private ElapsedTime runtime = new ElapsedTime();  // required for delay

    /*\
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newSpinnerTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setTargetPosition(leftInches, rightInches);
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drive(speed, speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.isDriving()) {
                // Allow time for other processes to run.
                idle();
            }

        }

        // Stop all motion;
        robot.drive(0,0);
        // Turn off RUN_TO_POSITION
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    private void setCatapultAndLaunch() throws InterruptedException
    {

        robot.catapultMotor.setTargetPosition(CATAPULT_LAUNCH_COUNT);
        robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.catapultMotor.setPower(.5);
        telemetry.addData("Catapult","Ready Position");
        telemetry.update();
        sleep(2000);
        robot.catapultMotor.setTargetPosition(FULL_ROTATION);
        robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Catapult", "Fired!");
        telemetry.update();
        sleep(2000);
        robot.catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Catapult", "Reset");
        telemetry.update();

    }
    */

    // attempts to place catapult in launch position (returns true if successful)
    private boolean readyCatapult() throws InterruptedException {
        robot.catapultMotor.setPower(0.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 10) && !robot.launchButton.getState()) {
            // Allow time for other processes to run.
            idle();
        }
        robot.catapultMotor.setPower(0.0);
        return robot.launchButton.getState();
    }

    private boolean launchCatapult() throws InterruptedException {
        robot.catapultMotor.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5) && robot.launchButton.getState()) {
            idle();
        }
        sleep(500);
        robot.catapultMotor.setPower(0.0);
        return robot.launchButton.getState();
    }

    private void loadCatapult() throws InterruptedException {
        sleep(500);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0) || ballLoaderOffset > ballLoaderTargetPos) {
            ballLoaderOffset -= 0.015;
            robot.Qermy.setPosition(ballLoaderOffset);
        }
        sleep(500);
        robot.Qermy.setPosition(ballLoaderDefaultPos);
        sleep(500);
    }
    private void findColorAndSetServo(double speed, double timeout) throws InterruptedException {

        // this logic assumes the color sensor is on the right
        pusherLeftPos = PUSHER_DOWN;
        pusherRightPos = PUSHER_DOWN;

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive(speed, speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout) && pusherLeftPos == pusherRightPos) {

            // this logic assumes the color sensor is on the right
            int color = robot.getColorNumber();

            if (color == SENSOR_RED) {
                robot.redLED(true);
                robot.blueLED(false);
                if (blueTeam) {
                    pusherRightPos = PUSHER_UP_RIGHT;
                } else {
                    pusherLeftPos = PUSHER_UP_LEFT;
                }

            } else if (color == SENSOR_BLUE) {
                robot.redLED(false);
                robot.blueLED(true);
                if (blueTeam) {
                    pusherLeftPos = PUSHER_UP_LEFT;
                } else {
                    pusherRightPos = PUSHER_UP_RIGHT;
                }

            } else {
                robot.redLED(false);
                robot.blueLED(false);

            }
            telemetry.addData("ColorNumber: %d", color);
            updateTelemetry(telemetry);
            idle();
        }
        robot.drive(0,0);
        robot.pusherLeft.setPosition(pusherLeftPos);
        robot.pusherRight.setPosition(pusherRightPos);
        sleep(50); // let servios move
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            sleep(10000);
            readyCatapult();
            launchCatapult();
            readyCatapult();
            loadCatapult();
            launchCatapult();
            encoderDrive(.4, 7, 7, 10); // move forward 7 inches
            encoderDrive(.4, 15.6, -15.6, 10); // Rotate approx 90°, approx 5.68° for each inch turned
            encoderDrive(.4, -55, -55, 10);
            /*old stuff
            encoderDrive(DRIVE_SPEED, 6, 6, 10); // move forward 6 inches
            encoderDrive(DRIVE_SPEED, -18.4, 18.4, 10); // Rotate approx approx 105, 5.68° for each inch turned
            encoderDrive(DRIVE_SPEED, 84, 84, 20);
            encoderDrive(DRIVE_SPEED, -6.25, 6.25, 10); // Rotate approx 35°, approx 5.68° for each inch turned
            encoderDrive(0.2, 10, 10, 10);
            //check color, raise button
            findColorAndSetServo(0.1,5.0);
            robot.drive(0.1, 0.1);
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                idle();
            }
            robot.drive(0, 0);*/


/*
            RobotLog.d("QbotAutonomousTest: OPMODE ACTIVE!");
            telemetry.addData("Status", "Active");
            telemetry.update();

            robot.catapultMotor.setTargetPosition(CATAPULT_LAUNCH_COUNT);
            robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.catapultMotor.setPower(.3);

            while (opModeIsActive() && robot.catapultMotor.isBusy())
            {
                telemetry.addData("Catapult", "Position %d", robot.catapultMotor.getCurrentPosition());
                telemetry.update();
                idle();
            }

            robot.catapultMotor.setPower(0);
            sleep(1000);

            robot.catapultMotor.setTargetPosition(FULL_ROTATION);
            robot.catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.catapultMotor.setPower(.3);

            while (opModeIsActive() && robot.catapultMotor.isBusy())
            {
                telemetry.addData("Catapult", "Position %d", robot.catapultMotor.getCurrentPosition());
                telemetry.update();
                idle();
            }

            robot.catapultMotor.setPower(0);
            robot.catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(2000);

        }

        telemetry.addData("Status", "Complete");
        telemetry.update();
        */
        }
    }

}
