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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Glyph Test", group="Test")
//@Disabled
public class AutoGlyphTest extends LinearOpMode {

    private static final int armMaxPos = 3000; // read encoder farthest travel position
    private static final double armPower = 0.25;

    DcMotor armMotor;
    private ElapsedTime runtime = new ElapsedTime();


    public void initArm()  {
        armMotor = hardwareMap.dcMotor.get("motor180");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
    }

    public void armMoveTo(int armPos)  {

         if (opModeIsActive()) {
             armPos = Range.clip( armPos, 0, armMaxPos );
             armMotor.setTargetPosition(armPos);
             armMotor.setPower( armPower );
            // keep looping while we are busy
            runtime.reset();
            while (opModeIsActive() && armMotor.isBusy()) {
                // Allow time for other processes to run.
                telemetry.addData("Arm position", armMotor.getCurrentPosition());
                telemetry.update();
                idle();
            }
            armMotor.setPower(0);
        }
    }

    @Override
    public void runOpMode() {

        initArm();
         // Declare OpMode members.

        telemetry.addData("Arm start", armMotor.getCurrentPosition());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armMoveTo( 500 );
        sleep(1000);
        armMoveTo( 2500 );
        sleep(1000);
        armMoveTo( 1500 );
        sleep(1000);
        armMoveTo( 0 );

        while (opModeIsActive() )
        {
            telemetry.addData("Arm position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
