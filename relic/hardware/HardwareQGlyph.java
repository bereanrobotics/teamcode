package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQGlyph {

    public DcMotor motor180 = null;
    public Servo glyphRight = null;
    public Servo glyphLeft  = null;

    HardwareMap hwMap   =  null;
    Telemetry telemetry = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareQGlyph (){}

    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }

    public void init(HardwareMap ahwMap, Telemetry telemetryPassed) {

        hwMap = ahwMap;
        telemetry = telemetryPassed;

        motor180 = initMotor(HardwareQConstants.GLYPH_ARM_MOTORNAME, false);
        glyphRight = initServo(HardwareQConstants.GLYPH_RIGHT_SERVONAME, 1, false);
        glyphLeft = initServo(HardwareQConstants.GLYPH_LEFT_SERVONAME, 0, false);

    }

}
