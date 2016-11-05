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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides  Telop driving for Aimbot.
 */

@TeleOp(name="QBot: Teleop", group="Qbot")
// @Disabled
public class QBotTeleop extends OpMode{

    /* Declare OpMode members. */

    HardwareQBot robot = new HardwareQBot(); // use the class created to define a Aimbot's hardware
    double qermyOffset = 0.7;
    boolean delayOn = false;
    boolean readyForTimerReset = true;
    private ElapsedTime     runtime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double catapultPower;
        double spinnerPower;
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        catapultPower = -gamepad2.left_stick_y;
        spinnerPower = gamepad2.left_stick_y;
        right = gamepad1.right_stick_y;
        left = gamepad1.left_stick_y;
        robot.spinner.setPower(spinnerPower/2);
        robot.catapultMotor.setPower(catapultPower/2);
        robot.front_right.setPower(right);
        robot.back_right.setPower (right);
        robot.front_left.setPower (right);
        robot.back_left.setPower (right);
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.left_bumper && !delayOn)
        {
            delayOn = true;
        }
        if (delayOn && qermyOffset > 0.25)
        {
            qermyOffset -= 0.01;
        }
        if (delayOn && qermyOffset <= 0.25)
        {
            if (readyForTimerReset)
            {
                runtime.reset();
                readyForTimerReset = false;
            }
            if (runtime.seconds() > 1)
            {
                delayOn = false;
            }

        }
        if (qermyOffset < 0.7 && !delayOn)
        {
            readyForTimerReset = true;
            qermyOffset += 0.01;
        }
        // Move both servos to new position.  Assume servos are mirror image of each other.
        robot.Qermy.setPosition(qermyOffset);

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("catapult", "catapult encoder: %d", robot.catapultMotor.getCurrentPosition());
        telemetry.addData("spinner", "right: %.2f", right);
        telemetry.addData("qermy", "%.2f", qermyOffset);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
