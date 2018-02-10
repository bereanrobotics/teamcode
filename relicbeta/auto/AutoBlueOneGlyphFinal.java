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

package org.firstinspires.ftc.teamcode.relicbeta.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.relic.hardware.HardwareRRQbot;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BLUE ONE: GLYPH", group="GLYPH")
@Disabled
public class AutoBlueOneGlyphFinal extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRRQbot robot = new HardwareRRQbot(); // use the class created to define a Aimbot's hardware
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.25;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        robot.glyph(robot.GLYPH_DROP);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.glyph(robot.GLYPH_GRAB);
        robot.pauseDriveTrain(0.5,telemetry);
        robot.moveArm(-0.5, 0.5, telemetry);

        robot.driveDiagonal(robot.FORWARD_RIGHT, 2.4, moveSpeed, telemetry);
        robot.pauseDriveTrain(0.5,telemetry);
        robot.rotateRobot(robot.LEFT, 0.6, moveSpeed, telemetry);
        pauseRobot(.5);
        straight(robot.FORWARD, 1.2);
        pauseRobot(.5);

        glyph(0);

        stopMoving();
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
            robot.backmotor.setPower(-moveSpeed);
            robot.frontmotor.setPower(moveSpeed);
            robot.rightmotor.setPower(moveSpeed);
            robot.leftmotor.setPower(-moveSpeed);
        } else
        {
            robot.backmotor.setPower(moveSpeed);
            robot.frontmotor.setPower(-moveSpeed);
            robot.rightmotor.setPower(-moveSpeed);
            robot.leftmotor.setPower(moveSpeed);
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
            robot.backmotor.setPower(moveSpeed);
            robot.frontmotor.setPower(moveSpeed);
            robot.rightmotor.setPower(moveSpeed);
            robot.leftmotor.setPower(moveSpeed);
        }

        if (direction == robot.FORWARD_RIGHT)
        {
            robot.backmotor.setPower(-moveSpeed);
            robot.frontmotor.setPower(-moveSpeed);
            robot.rightmotor.setPower(moveSpeed);
            robot.leftmotor.setPower(moveSpeed);
        }

        if (direction == robot.BACKWARD_LEFT)
        {
            robot.backmotor.setPower(moveSpeed);
            robot.frontmotor.setPower(moveSpeed);
            robot.rightmotor.setPower(-moveSpeed);
            robot.leftmotor.setPower(-moveSpeed);
        }

        if (direction == robot.BACKWARD_RIGHT)
        {
            robot.backmotor.setPower(-moveSpeed);
            robot.frontmotor.setPower(-moveSpeed);
            robot.rightmotor.setPower(-moveSpeed);
            robot.leftmotor.setPower(-moveSpeed);
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

            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(moveSpeed);
            robot.leftmotor.setPower(moveSpeed);
        }

        if (direction == robot.BACKWARD)
        {
            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(-moveSpeed);
            robot.leftmotor.setPower(-moveSpeed);
        }

        while (opModeIsActive() && ((runtime.seconds()-startTime) < straightTime))
        {
            telemetry.addData("Status", "driving straight");
            telemetry.update();
        }

        stopMoving();
    }

    private void glyph ( double position )
    {
        robot.glyphLeft.setPosition(robot.MID_SERVO + position);
        robot.glyphRight.setPosition(robot.MID_SERVO - position);
    }

    private void rack( double power, double time )
    {
        double startTime = runtime.seconds();

        robot.motorRack.setPower( power);
        while (opModeIsActive() && ((runtime.seconds()-startTime) < time))
        {
            telemetry.addData("Status", "moveRack");
            telemetry.update();
        }
        robot.motorRack.setPower( 0 );
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
