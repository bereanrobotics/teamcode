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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.relicbeta.hardware.HardwareJewelArm;
import org.firstinspires.ftc.teamcode.relicbeta.hardware.HardwareQBot;


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

@Autonomous(name="BLUE TWO: theta", group="FINAL")
//@Disabled
public class AutoBlueTwo extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareQBot robot    = new HardwareQBot(); // use the class created to define a Aimbot's hardware
    HardwareJewelArm jArm = new HardwareJewelArm();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.25;
    private int directionRotated;
    private int directionToDrive;
    public static final int BLUE  = 94;
    public static final int RED   = 49;
    public static final int ONE   = 1;
    public static final int TWO   = 2;
    public static final int THREE = 3;
    private int positionNumber;

    /////////////

    private int teamColor = BLUE;
    private int teamPosition = TWO;

    /////////////

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        jArm.init(hardwareMap);
        robot.glyphLeft.setPosition(0);
        robot.glyphRight.setPosition(1);

        telemetry.addData("Status", "1");

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        sleep(1000);
        telemetry.addData("Servo Position", jArm.jewelArmLift.getPosition());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        positionNumber = getPostionNo();
        positionNumber = ONE; //delete this line once vuforia works
        robot.glyphLeft.setPosition(1);
        robot.glyphRight.setPosition(0);
        robot.motor180.setPower(.5);


        knockJewel();
        pauseRobot(1);

        telemetry.addData("Status", "2");
        telemetry.update();

        getDirectionDrive(directionRotated);
        diagonal(directionToDrive, 2.2); //2.2 is an estimate
        pauseRobot(1);

        telemetry.addData("Status", "3");
        telemetry.update();

        parallel(); //makes robot parallel to the glyph case
        delivierBlock();
        straight(robot.FORWARD, 1.2); // add code afterwards to lower the arm into the slot

        /*
        diagonal(robot.FORWARD_RIGHT, 2.4);
        pauseRobot(.5);
        rotateRobot(robot.LEFT, .6);
        pauseRobot(.5);
        straight(robot.FORWARD, 1.2);
        pauseRobot(.5);*/

        stopMoving();
    }

    private void knockJewel ()
    {
        int jewelColor;

            jArm.deploy();
            pauseRobot(4);
            //sleep(2000);

            jewelColor = getColor();

            // we should use robotlog to write what it finds to the log for post tournament eval

            if (teamColor != jewelColor) //looking at my jewel, so knock right
            {
                rotateRobot(robot.RIGHT, robot.turnRotate);
                directionRotated = robot.RIGHT;

            } else
            {
                rotateRobot(robot.LEFT, robot.turnRotate);
                directionRotated = robot.LEFT;
            }

            jArm.retract();


    }

    private void parallel ()
    {
         if     ((directionRotated == robot.RIGHT) && (teamPosition == ONE))
         {
             rotateRobot(robot.RIGHT, robot.turnParallel);
         }
        else if ((directionRotated == robot.LEFT)  && (teamPosition == TWO))
        {
            rotateRobot(robot.RIGHT, robot.turnParallel);
        } else
        {
            rotateRobot(robot.LEFT, robot.turnParallel);
        }
    }

    private void delivierBlock ()
    {
        if (teamColor == RED)
        {
            straight(robot.LEFT, .2); //.2 is an estimate
            if (positionNumber == TWO)
            {
                straight(robot.LEFT, .4); //.4 is an estimate
            }
            if (positionNumber == THREE)
            {
                straight(robot.LEFT, .4);
            }
        } else
        {
            straight(robot.RIGHT, .2);
            if (positionNumber == TWO)
            {
                straight(robot.RIGHT, .4);
            }
            if (positionNumber == THREE)
            {
                straight(robot.RIGHT, .4);
            }
        } 
        straight(robot.FORWARD, robot.driveOut); // add code afterwards to lower the arm into the slot
    }

    private int getPostionNo ()
    {
        return THREE;
    }

    private int getColor()
    {
        int blueValue;
        int redValue;

        blueValue = jArm.sensorColor.blue();
        redValue = jArm.sensorColor.red();
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);
        telemetry.update();
        if (blueValue > redValue) {
            return BLUE;
        }
        else {
            return RED;
        }
    }

    private void getDirectionDrive(int directionRotate)
    {
        if (teamColor == RED) {
            if (directionRotate == robot.RIGHT) {
                directionToDrive = robot.FORWARD_RIGHT;
            } else {
                directionToDrive = robot.BACKWARD_RIGHT;
            }
        } else if (teamColor == BLUE)
        {
            if (directionRotate == robot.RIGHT) {
                directionToDrive = robot.BACKWARD_LEFT;
            } else {
                directionToDrive = robot.FORWARD_LEFT;
            }
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
