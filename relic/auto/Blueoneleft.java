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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQConstants;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQDrive;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQGlyph;
import org.firstinspires.ftc.teamcode.relic.hardware.HardwareQJewelArm;

/**
 * Created by BCHSRobotics1 on 2/10/2018.
 */

@Autonomous(name="BlueOneLeft", group="FINAL")

public class Blueoneleft extends LinearOpMode{
    HardwareQDrive    drive     = new HardwareQDrive(); // use the class created to define a Aimbot's hardware
    HardwareQGlyph    gArm      = new HardwareQGlyph();
    HardwareQJewelArm jArm      = new HardwareQJewelArm();
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.20;
    private int slotNum;
    private int colorBasedDrive;

    private int teamColor = HardwareQConstants.BLUE;
    private int teamPosition = 1 ;

    private VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public void VuForiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac80Bxv/////AAAAGfuMJvbL4EHBo0m/sdW89cEZQtlemG/Mzd5+DnP8zVE+fzNuHuRvUQTpoNwJ6/rkPwmuMiCAzOmspCyheyCAl5OLh7Xbp95m1wWcdjb/kUAdtPsfNel0eNn2ji0iwoWuPtQJ+b8YLFSyzBCkHe9dB05c86KmdvJHAH7+y//rLsAEy5/josOhopghBAEJGUjjwWe5n/I0Fe2Hmpv1Kw8SuhCFp3ibSL/YoCRYrRx0lKQSQ7ZdlcEVO2uVAMpStijUn1rWNlp6WaaZ5+Z1Su8IVWRll/5xFZGs/vdcxpqbJbGXKfZsQE1vYSH6rPc/w0cvBJW09GHxnburccDC5ZBTva63VGbAhBWlkp/MXUvlN9Q5";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    public void armMoveTo(int armPos)  {

        if (opModeIsActive()) {
            gArm.motor180SetPosition( armPos );
            // keep looping while we are busy
            runtime.reset();
            while (opModeIsActive() && gArm.motor180.isBusy()) {
                // Allow time for other processes to run.
                telemetry.addData("Arm position", gArm.motor180.getCurrentPosition());
                telemetry.update();
                idle();
            }
            gArm.motor180.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        drive.init(hardwareMap, telemetry);
        gArm.init (hardwareMap, telemetry);
        jArm.init (hardwareMap, telemetry);

        //VuForiaInit();
        //gArm.glyphRight.setPosition(1); // moving the jewel
        //gArm.glyphLeft.setPosition(0);  // arm to the init pos
         //... add code move glyph arm to init position too
        telemetry.addData("Status", "Initialized");

        waitForStart();

        //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
           ///set slotnum here...
       // }

        runtime.reset();

        gArm.glyphRight.setPosition(1); // picking up
        gArm.glyphLeft.setPosition(0);  // the glyph

        pauseRobot(2);
//        armMoveTo(650);
        //... raise arm a tiny bit

        slotNum = 1; //replace with vuforia

        knockJewel();
        pauseRobot(3);


        colorBasedDrive = colorToDirection();
        drive.strafeFor(colorBasedDrive, 4.35, speed); //get to incrementing point
        if (teamPosition == 2) {
            drive.rotateRobot(colorBasedDrive, .5, speed); // rotate 45 degrees to be parallel to goal
        }

        drive.rotateRobot(HardwareQConstants.RIGHT, 5.6, 0.15);

        //increment();
        drive.strafeFor(HardwareQConstants.FORWARD, .5, speed );
        //armMoveTo( gArm.motor180MaxPosition );

        gArm.glyphRight.setPosition(0);
        gArm.glyphLeft.setPosition(1);
        pauseRobot(1.5);
        gArm.glyphRight.setPosition(1);
        gArm.glyphLeft.setPosition(0);
        pauseRobot(1.5);
        gArm.glyphRight.setPosition(0);
        gArm.glyphLeft.setPosition(1);
        pauseRobot(1.5);
        gArm.glyphLeft.setPosition(0);
        gArm.glyphLeft.setPosition(1);


        drive.strafeFor(HardwareQConstants.FORWARD, 1.5, speed );
        //sleep(500);
        //armMoveTo( 100 );
        drive.strafeFor(HardwareQConstants.BACKWARD, 0.5, speed );

       /* drive.strafeFor(HardwareQConstants.FORWARD, 3, speed);
        drive.strafeFor(HardwareQConstants.RIGHT, 3, speed);
        drive.strafeFor(HardwareQConstants.BACKWARD, 3, speed);
        drive.strafeFor(HardwareQConstants.LEFT, 3, speed); */
    }

    private void knockJewel() {
        jArm.deploy();

        sleep(4000);
        if (getColor() != teamColor) {
            drive.rotateRobot(HardwareQConstants.LEFT, .5, .14); //rotate 180 degrees
            jArm.retract();
            pauseRobot(.5);
            drive.rotateRobot(HardwareQConstants.RIGHT, .5, .13); //rotate 180 degrees
            // exchange previous func for encoder version
        } else {
            drive.rotateRobot(HardwareQConstants.RIGHT, .5, .13);
            jArm.retract();
            pauseRobot(.5);
            drive.rotateRobot(HardwareQConstants.LEFT, .5, .14); //rotate 180 degrees
        }

    }

    private void increment() {
        if (slotNum == 1) {
            drive.strafeFor(colorBasedDrive, 0.1, speed);
        } else if (slotNum == 2) {
            drive.strafeFor(colorBasedDrive, 0.4, speed);
        } else if (slotNum == 3) {
            drive.strafeFor(colorBasedDrive, 0.4, speed);
        }
    }
    private int getColor()
    {
        int blueValue;
        int redValue;

        blueValue = jArm.sensorColor.blue();
        redValue = jArm.sensorColor.red();
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);
        if(blueValue > redValue) {
            telemetry.addData("Result", "Blue");
            telemetry.update();
            return HardwareQConstants.BLUE;
        }
        else {
            telemetry.addData("Result", "Red");
            telemetry.update();
            return HardwareQConstants.RED;
        }
    }
    private int colorToDirection () {
        return ((teamColor == HardwareQConstants.BLUE) ?
        HardwareQConstants.LEFT : HardwareQConstants.RIGHT);
    }

    private void pauseRobot(double pauseSeconds)
    {
        double startTime = runtime.seconds();

        drive.stopMoving();

        while (opModeIsActive() && ((runtime.seconds()-startTime) < pauseSeconds))
        {
            telemetry.addData("Status", "pausing");
            telemetry.update();
        }

    }


}
