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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides  Telop driving for Aimbot.
 */

@Autonomous(name="MiniBot: AutoColor2", group="mini")
@Disabled
public class MiniBotAutoColor extends LinearOpMode{

    /* Declare OpMode members. */

    HardwareAimbot robot = new HardwareAimbot(); // use the class created to define a Aimbot's hardware

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    static final int        SENSOR_RED     = 10;
    static final int        SENSOR_BLUE    = 3;

    static final double     PUSHER_DOWN    = 0.0;
    static final double     PUSHER_UP      = 0.5;

    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light

    Boolean redTeam = true; // flag to determine if we are the red or blue team.

    private ElapsedTime runtime = new ElapsedTime();  // required for delay

    // delay routine that correctly handles opMode logic
    private void delay(double seconds) throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Delay", "T:%2.5f", runtime.seconds());
            telemetry.update();
            idle();
        }
    }

    private void drive(double left, double right, double seconds) throws InterruptedException {
        if (opModeIsActive()) {
            robot.drive(left, right);
            delay(seconds);
        }
        //    telemetry.addData("Drive", "L:%2.2f, R:%2.2f, T:%2.5f", left, right, runtime.seconds());
    }

    /*
    private void driveToLine(double left, double right, double seconds) throws InterruptedException {
        if (opModeIsActive()) {
            robot.drive(left, right);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds) &&
                    (robot.lightSensor.getLightDetected() < WHITE_THRESHOLD)) {
                telemetry.addData("Light Level", robot.lightSensor.getLightDetected());
                telemetry.addData("Delay", "T:%2.5f", runtime.seconds());
                telemetry.update();
                idle();
            }
        }
    }

*/
    private void park() {
        robot.drive(0,0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        double pusherRightPos = PUSHER_DOWN;
        double pusherLeftPos = PUSHER_DOWN;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //drive(FORWARD_SPEED, FORWARD_SPEED, 1.0);   // Step 1:  Drive forward for 1 seconds
        //drive(TURN_SPEED, -TURN_SPEED, 1.0);        // Step 2:  Spin right for 1.3 seconds
        //drive(-FORWARD_SPEED, -FORWARD_SPEED, 1.0); // Step 3:  Drive Backwards for 1 Second
        //park();                                      // Step 4: Stop


        while(opModeIsActive()) {

            int color = robot.getColorNumber();

            // this logic assumes the color sensor is on the right
            pusherLeftPos = PUSHER_DOWN;
            pusherRightPos = PUSHER_DOWN;

            if (color == SENSOR_RED) {
                robot.redLED(true);
                robot.blueLED(false);
                if (redTeam) {
                    pusherRightPos = PUSHER_UP;
                } else {
                    pusherLeftPos = PUSHER_UP;
                }

            } else if (color == SENSOR_BLUE) {
                robot.redLED(false);
                robot.blueLED(true);
                if (redTeam) {
                    pusherLeftPos = PUSHER_UP;
                } else {
                    pusherRightPos = PUSHER_UP;
                }

            } else {
                robot.redLED(false);
                robot.blueLED(false);

            }

            robot.leftButtonPusher.setPosition(pusherLeftPos);
            robot.rightButtonPusher.setPosition(pusherRightPos);

            telemetry.addData("ColorNumber: %d", color);
            updateTelemetry(telemetry);
            sleep(50);
            idle();
        }


        /*robot.frontLeftMotor.setPower(FORWARD_SPEED);
        robot.frontRightMotor.setPower(FORWARD_SPEED);
        sleep(500);
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);*/
    }

}
