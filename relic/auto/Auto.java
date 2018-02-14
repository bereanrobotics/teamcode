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

package org.firstinspires.ftc.teamcode.relic.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQConstants;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQDrive;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQGlyph;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQJewelArm;

/**
 * Created by BCHSRobotics1 on 2/10/2018.
 */

@Autonomous(name="Test", group="FINAL")

public class Auto extends LinearOpMode{
    HardwareQDrive drive        = new HardwareQDrive(); // use the class created to define a Aimbot's hardware
    HardwareQGlyph gArm         = new HardwareQGlyph();
    HardwareQJewelArm jArm      = new HardwareQJewelArm();
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.25;

    private int teamColor;
    private int teamPosition;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap, telemetry);
        gArm.init (hardwareMap, telemetry);
        jArm.init (hardwareMap, telemetry);

        gArm.glyphRight.setPosition(1);
        gArm.glyphLeft.setPosition(0);

        waitForStart();
        runtime.reset();

        drive.strafeFor(HardwareQConstants.FORWARD, 3, speed);
        drive.strafeFor(HardwareQConstants.RIGHT, 3, speed);
        drive.strafeFor(HardwareQConstants.BACKWARD, 3, speed);
        drive.strafeFor(HardwareQConstants.LEFT, 3, speed);
    }


}
