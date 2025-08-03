package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //drive motor
    public DcMotorEx frontRight, backRight, backLeft, frontLeft;
    //intakemotor
    public DcMotorEx intakeMotor;
    //servos
    public Servo xServo, yServo, rotateServo, clawServo;



    public RobotHardware(HardwareMap hwMap) {
        // TODO: Add in fail-first code
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");  // CH3
        backRight  = hwMap.get(DcMotorEx.class, "backRight");   // CH2
        backLeft   = hwMap.get(DcMotorEx.class, "backLeft");    // CH1
        frontLeft  = hwMap.get(DcMotorEx.class, "frontLeft");   // CH0



        frontLeft .setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft  .setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight .setDirection(DcMotorSimple.Direction.REVERSE);
        //motors have to be reverse becuz they are mounted backwards in cad

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //this makes it not move and resets encoders then gives feedback


        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        xServo = hwMap.get(Servo.class, "xServo");
        yServo = hwMap.get(Servo.class, "yServo");
        rotateServo = hwMap.get(Servo.class, "rotateServo");
        clawServo = hwMap.get(Servo.class, "clawServo");

/*
        xServo.setPosition(0.5);
        yServo.setPosition(0.5);
        rotateServo.setPosition(0.5);
        clawServo.setPosition(0.0);
*/

    }
}
