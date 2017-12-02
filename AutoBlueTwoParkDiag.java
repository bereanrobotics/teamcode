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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="BLUE TWO: PARK", group="FINAL")
//@Disabled
public class AutoBlueTwoParkDiag extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareQ4Way robot = new HardwareQ4Way(); // use the class created to define a Aimbot's hardware
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.25;
    private double forwardTime = 0.4 ;
    private double sideTime = 2.4;
    private int LEFT = 1;
    private int RIGHT = 2;
    private int FORWARD_LEFT = 3;
    private int FORWARD_RIGHT = 4;
    private int BACKWARD_LEFT = 5;
    private int BACKWARD_RIGHT = 6;
    private int FORWARD = 7;
    private int BACKWARD = 8;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        diagonal(FORWARD_RIGHT, 2.3);
        pauseRobot(.5);
        diagonal(BACKWARD_RIGHT,1.0);
        pauseRobot(.5);
        rotateRobot(RIGHT,.5);

        pauseRobot(.5);

        robot.backmotor.setPower(0);
        robot.frontmotor.setPower(0);
        robot.rightmotor.setPower(0);
        robot.leftmotor.setPower(0);
    }

    private void pauseRobot(double pauseSeconds)
    {
        double startTime = runtime.seconds();

        robot.backmotor.setPower(0);
        robot.frontmotor.setPower(0);
        robot.rightmotor.setPower(0);
        robot.leftmotor.setPower(0);

        while (opModeIsActive() && ((runtime.seconds()-startTime) < pauseSeconds))
        {
            telemetry.addData("Status", "pausing");
            telemetry.update();
        }

    }

    private void rotateRobot(int direction, double rotateSeconds)
    {
        double startTime = runtime.seconds();

        if(direction == LEFT)
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
    }

    private void diagonal (int direction, double moveSeconds)
    {
        double startTime = runtime.seconds();

        if (direction == FORWARD_LEFT)
        {
            robot.backmotor.setPower(speed);
            robot.frontmotor.setPower(speed);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == FORWARD_RIGHT)
        {
            robot.backmotor.setPower(-speed);
            robot.frontmotor.setPower(-speed);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == BACKWARD_LEFT)
        {
            robot.backmotor.setPower(speed);
            robot.frontmotor.setPower(speed);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(-speed);
        }

        if (direction == BACKWARD_RIGHT)
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
    }

    private void straight (int direction, double straightTime)
    {
        double startTime = runtime.seconds();

        if (direction == FORWARD)
        {
            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(speed);
            robot.leftmotor.setPower(speed);
        }

        if (direction == BACKWARD)
        {
            robot.backmotor.setPower(0);
            robot.frontmotor.setPower(0);
            robot.rightmotor.setPower(-speed);
            robot.leftmotor.setPower(-speed);
        }

        while (opModeIsActive() && ((runtime.seconds()-startTime) < straightTime))
        {
            telemetry.addData("Status", "driving straight");
            telemetry.update();
        }
    }
}
