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
package org.firstinspires.ftc.teamcode.relicbeta.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.relicbeta.hardware.HardwareQBot;


/**
 * This file provides  Telop driving for Aimbot.
 */

@TeleOp(name="TeleOpTest", group="Q")
// @Disabled

public class TeleopTestx extends OpMode{

    /* Declare OpMode members. */
    HardwareQBot robot = new HardwareQBot(); // use the class created to define a Aimbot's hardware
    private double speedFactor = 1;
    private boolean sniperMode = false;
    private double maxWheelSpeed = 0.80;

    public static final double POWER_FACTOR_RACK = .25;
    public static final double POWER_FACTOR_180 = 1;
    static double CLAW_SPEED = 0.4;

    double glyphGrabber = 0;

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
        double left;
        double right;
        double front;
        double back;
        double m180;
        //double rack;

        final double    CLAW_SPEED  = 0.04 ;                 // sets rate to move servo


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        front = -gamepad1.left_stick_x;
        back = -gamepad1.right_stick_x;

        if (gamepad1.dpad_up)
        {

            left = maxWheelSpeed;
            right= maxWheelSpeed;
            back = 0;
            front = 0;
        }

        if (gamepad1.dpad_down)
        {


            left = -maxWheelSpeed;
            right= -maxWheelSpeed;
            back = 0;
            front = 0;
        }

        if (gamepad1.dpad_left)
        {

            left = 0;
            right= 0;
            back = -maxWheelSpeed;
            front = -maxWheelSpeed;
        }
        if (gamepad1.dpad_right)
        {


            left = 0;
            right= 0;
            back = maxWheelSpeed;// negative means right
            front = maxWheelSpeed; // positive means left
        }

        if (gamepad1.left_bumper)
        {

            left = -maxWheelSpeed;
            right= maxWheelSpeed;
            back = maxWheelSpeed;
            front = -maxWheelSpeed;
        }
        if (gamepad1.right_bumper)
        {

            left = maxWheelSpeed;
            right= -maxWheelSpeed;
            back = -maxWheelSpeed;
            front = maxWheelSpeed;
        }

        if (gamepad1.right_stick_button)
        {

            speedFactor = -1;

        }
        if (gamepad1.left_stick_button)
        {

            speedFactor = 1;

        }
        if (gamepad1.y)
        {
            sniperMode = true;
        }
        if (gamepad1.x)
        {
            sniperMode = false;
        }

        if(sniperMode)
            speedFactor = .5;
        else
            speedFactor = 1;

        robot.leftmotor.setPower(left * speedFactor);
        robot.backmotor.setPower(back * speedFactor);
        robot.rightmotor.setPower(right * speedFactor);
        robot.frontmotor.setPower(front * speedFactor);

        //note: The joystick goes negative when pushed forwards, so negate it)
        m180 = -gamepad2.left_stick_y;
        //rack = -gamepad2.right_stick_y;

        //robot.motorRack.setPower(rack * POWER_FACTOR_RACK);
        robot.motor180.setPower(m180 * POWER_FACTOR_180);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.right_bumper)
            glyphGrabber += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            glyphGrabber -= CLAW_SPEED;


        // Move both servos to new position.  Assume servos are mirror image of each other.
        glyphGrabber = Range.clip(glyphGrabber, -0.5, 0.5);
        robot.glyphLeft.setPosition(robot.MID_SERVO + glyphGrabber);
        robot.glyphRight.setPosition(robot.MID_SERVO - glyphGrabber);


        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("glyph", "%.2f", glyphGrabber);
        telemetry.addData("arm", "%.2f", m180);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
