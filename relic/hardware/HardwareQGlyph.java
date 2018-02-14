package org.firstinspires.ftc.teamcode.relic.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.BereanHardware;

/**
 * Created by BCHSRobotics1 on 2/6/2018.
 */

public class HardwareQGlyph extends BereanHardware {

    public DcMotor motor180 = null;
    public Servo glyphRight = null;
    public Servo glyphLeft  = null;
    public static final double MID_SERVO       =  0.5 ;

    private int motor180MaxPosition = 3000;
    private double motor180Power = 0.25;

    public HardwareQGlyph (){}

    public void init(HardwareMap ahwMap, Telemetry telemetryPassed) {

        super.init(ahwMap,telemetryPassed);

        motor180 = initMotor(HardwareQConstants.GLYPH_ARM_MOTORNAME, false);
        glyphRight = initServo(HardwareQConstants.GLYPH_RIGHT_SERVONAME, 1, false);
        glyphLeft = initServo(HardwareQConstants.GLYPH_LEFT_SERVONAME, 0, false);

    }

    // moves motor180 to a given position in its range
    // should convert this to a generic encoder move routine
    public void motor180SetPosition( int armPos ) {
        armPos = Range.clip( armPos, 0, motor180MaxPosition );
        if ( armPos != motor180.getTargetPosition() ) {
            motor180.setTargetPosition( armPos );
            motor180.setPower( motor180Power );
        } else if ( !motor180.isBusy() ) {
            motor180.setPower( 0 );
        }
    }

}
