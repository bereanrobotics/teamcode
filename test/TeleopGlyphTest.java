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
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This file provides  Telop test for glyph arm controlled by encoder.
 */

@TeleOp(name="TeleOpGlyphTest", group="Test")
// @Disabled

public class TeleopGlyphTest extends OpMode{

    private static final int armMaxPos = 3000; // read encoder farthest travel position
    private DcMotor armMotor;
    private int armPos = -1; // should always be positive after init
    private int armStartPos = 100;  // if we want an initial lift for the arm
    private double armPower = 0;
     /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        armMotor = hardwareMap.dcMotor.get("motor180");
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);

        telemetry.addData("Arm start", armMotor.getCurrentPosition());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (armPos == -1) {  // set current position to zero
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        armPos = armMotor.getCurrentPosition();
        telemetry.addData("Arm intitialize", armPos );
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setTargetPosition( armStartPos);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

         //note: The joystick goes negative when pushed forwards, so negate it)
        double m180 = -gamepad2.left_stick_y;
        int increment = (int) Math.floor( m180 * 100 );
        armPos = armMotor.getCurrentPosition() + increment;
        armPos = Range.clip( armPos, 0, armMaxPos );
        if ( armPos != armMotor.getTargetPosition() ) {
            armMotor.setTargetPosition( armPos );
            armPower = 0.25;
            armMotor.setPower( armPower );
        } else if ( !armMotor.isBusy() ) {  // && ( armMotor.getCurrentPosition() == armPos )){
            armPower = 0;
            armMotor.setPower( armPower );
        }

        telemetry.addData("arm", "increment %d, power %.2f, busy= %s", increment, armPower, armMotor.isBusy()? "busy" : "" );
        telemetry.addData("arm", "cur=%d, target=%d, pos=%d", armMotor.getCurrentPosition(), armMotor.getTargetPosition(), armPos );
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
