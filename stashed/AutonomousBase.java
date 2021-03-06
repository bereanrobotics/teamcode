/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.stashed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.relicbeta.hardware.HardwareJewelArm;
import org.firstinspires.ftc.teamcode.relicbeta.hardware.HardwareQBot;


/**
 * This is a base class for Relic Recovery Autonomous opcodes
 *
 **/


public class AutonomousBase extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareQBot robot    = new HardwareQBot(); // use the class created to define a Aimbot's hardware
    HardwareJewelArm jArm = new HardwareJewelArm();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.25;
    private static final int BLUE = 94;
    //public static final int RED  = 49;

    public int teamColor;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        jArm.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        telemetry.addData("Servo Position", jArm.jewelArmLift.getPosition());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //getTeamColor();
        teamColor = BLUE;

        knockJewel();

        diagonal(robot.FORWARD_RIGHT, 2.4);
        pauseRobot(.5);
        rotateRobot(robot.LEFT, .6);
        pauseRobot(.5);
        straight(robot.FORWARD, 1.2);
        pauseRobot(.5);

        stopMoving();
    }

    private void knockJewel ()
    {
        if (teamColor == BLUE)
        {
            jArm.jewelArmLift.setPosition(1);
            sleep(600);
            if ((jArm.sensorColor.blue()> 128)&&(jArm.sensorColor.red() < 126)) //knock of red
            {
                rotateRobot(robot.RIGHT, .6);
                rotateRobot(robot.LEFT, .6);
            } else
            {

            rotateRobot(robot.LEFT, .6);
            rotateRobot(robot.RIGHT, .6);
            }

            jArm.jewelArmLift.setPosition(.5);
        }

    }

    private void pauseRobot(double pauseSeconds)
    {
        double startTime = runtime.seconds();

        stopMoving();

        while (opModeIsActive() && ((runtime.seconds()-startTime) < pauseSeconds))
        {
            telemetry.addData("Status", "pausing");
            telemetry.update();
        }

    }

    private void rotateRobot(int direction, double rotateSeconds)
    {
        double startTime = runtime.seconds();

        if(direction == robot.LEFT)
        {
            robot.backmotor.setPower(-speed);
            robot.frontmotor.setPower(speed);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(-speed);
        } else
        {
            robot.backmotor.setPower(speed);
            robot.frontmotor.setPower(-speed);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(speed);
        }

        while (opModeIsActive() && ((runtime.seconds()-startTime) < rotateSeconds))
        {
            telemetry.addData("Status", "rotating");
            telemetry.update();
        }

        stopMoving();
    }

    private void diagonal (int direction, double moveSeconds)
    {
        double startTime = runtime.seconds();

        if (direction == robot.FORWARD_LEFT)
        {
            robot.backmotor.setPower(speed);
            robot.frontmotor.setPower(speed);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == robot.FORWARD_RIGHT)
        {
            robot.backmotor.setPower(-speed);
            robot.frontmotor.setPower(-speed);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == robot.BACKWARD_LEFT)
        {
            robot.backmotor.setPower(speed);
            robot.frontmotor.setPower(speed);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(-speed);
        }

        if (direction == robot.BACKWARD_RIGHT)
        {
            robot.backmotor.setPower(-speed);
            robot.frontmotor.setPower(-speed);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(-speed);
        }

        while (opModeIsActive() && ((runtime.seconds()-startTime) < moveSeconds))
        {
            telemetry.addData("Status", "driving diagonally");
            telemetry.update();
        }

        stopMoving();
    }

    private void straight (int direction, double straightTime)
    {
        double startTime = runtime.seconds();

        if (direction == robot.FORWARD)
        {

            telemetry.addData("Status", "driving straight");
            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == robot.BACKWARD)
        {
            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(-speed);
        }

        while (opModeIsActive() && ((runtime.seconds()-startTime) < straightTime))
        {   telemetry.update();
        }

        stopMoving();
    }

    private void glyph ( double position )
    {
        robot.glyphLeft.setPosition(robot.MID_SERVO + position);
        robot.glyphRight.setPosition(robot.MID_SERVO - position);
    }

    private void m180( double power, double time )
    {
        double startTime = runtime.seconds();

        robot.motor180.setPower( power);
        while (opModeIsActive() && ((runtime.seconds()-startTime) < time))
        {
            telemetry.addData("Status", "moveRack");
            telemetry.update();
        }
        robot.motor180.setPower( 0 );
    }

    private void stopMoving()
    {
        robot.backmotor.setPower(0);
        robot.frontmotor.setPower(0);
        robot.rightmotor.setPower(0);
        robot.leftmotor.setPower(0);
    }
}
