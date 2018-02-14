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
package org.firstinspires.ftc.teamcode.relic.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQConstants;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQDrive;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQGlyph;


/**
 * This file provides  Telop driving for Q' Relic Recover robot with
 * the mechanum chassis
 */

@TeleOp(name="TeleOpQ-Mechanum", group="Q")
// @Disabled

public class TeleOpQbotMechanum extends OpMode{

    /* Declare OpMode members. */
    //HardwareQBot robot = new HardwareQBot();
    private HardwareQDrive driveTrain = new HardwareQDrive(); // the robot module for the drive train
    private HardwareQGlyph glyphArm = new HardwareQGlyph(); // the robot module for the glyph arm
    private double speedFactor     = 1;
    private double directionFactor = 1;
    private double maxWheelSpeed = 0.80;
    private boolean strafeMode = false;

    //public static final double POWER_FACTOR_180 = 1;
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
        //robot.init(hardwareMap);
        driveTrain.init(hardwareMap,telemetry);
        glyphArm.init(hardwareMap,telemetry);

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
        // robot's left side
        //
        // control using left and right sticks.
        // speed set based on stick position
        //
        left = -gamepad1.left_stick_y;
        back = left;
        right = -gamepad1.right_stick_y;
        front = right;

        if ( left != 0 || right != 0)
            strafeMode = false;

        if (gamepad1.right_stick_button)
        {
            directionFactor = -1;
        }
        if (gamepad1.left_stick_button)
        {
            directionFactor = 1;
        }

        if (gamepad1.y)
        {
            speedFactor = .5;
        }
        if (gamepad1.x)
        {
            speedFactor = 1;
        }

        // control using the dpad.  strafing mode left and right with forward/back
        // fixed speed
        //
        // strafe forward
        if (gamepad1.dpad_up)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.FORWARD,maxWheelSpeed * speedFactor);
        }

        //strafe backward
        if (gamepad1.dpad_down)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.BACKWARD, maxWheelSpeed * speedFactor);
        }

        //strafe left
        if (gamepad1.dpad_left)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.LEFT, maxWheelSpeed * speedFactor);
        }

        //strafe right
        if (gamepad1.dpad_right)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.RIGHT, maxWheelSpeed * speedFactor);
        }

        //strafe left
        if (gamepad1.left_bumper)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.LEFT, maxWheelSpeed * speedFactor);
        }

        //strafe right
        if (gamepad1.right_bumper)
        {
            strafeMode = true;
            driveTrain.strafe(HardwareQConstants.RIGHT, maxWheelSpeed * speedFactor);
        }

        if (!strafeMode) // you must be controlling with the sticks
        {
            driveTrain.leftfrontmotor.setPower(left * speedFactor);
            driveTrain.leftbackmotor.setPower(back * speedFactor);
            driveTrain.rightfrontmotor.setPower(right * speedFactor);
            driveTrain.rightbackmotor.setPower(front * speedFactor);
        } else // if you aren't using sticks or the other options, make sure to stop
            if (!gamepad1.dpad_up &&
                !gamepad1.dpad_down &&
                !gamepad1.dpad_right &&
                !gamepad1.dpad_left &&
                !gamepad1.left_bumper &&
                !gamepad1.right_bumper) driveTrain.stopMoving();

        //note: The joystick goes negative when pushed forwards, so negate it)
        m180 = -gamepad2.left_stick_y;
        int increment = (int) Math.floor( m180 * 100 );
        int armPos = glyphArm.motor180.getCurrentPosition() + increment;
        glyphArm.motor180SetPosition( armPos );

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.right_bumper)
            glyphGrabber += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            glyphGrabber -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        glyphGrabber = Range.clip(glyphGrabber, -0.5, 0.5);
        glyphArm.glyphLeft.setPosition(glyphArm.MID_SERVO + glyphGrabber);
        glyphArm.glyphRight.setPosition(glyphArm.MID_SERVO - glyphGrabber);


        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("strafe?  ", strafeMode);
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
