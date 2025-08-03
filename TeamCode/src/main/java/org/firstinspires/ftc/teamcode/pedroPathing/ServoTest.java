package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ServoTest", group = "Test")
@Config
public class ServoTest extends OpMode {

    private Servo xServo, yServo, rotateServo, clawServo;

    public static double xPos = 0.9;
    public static double yPos = 0.9;
    public static double rotatePos = 0.5;
    public static double clawPos = 0.3;

    @Override
    public void init() {
        xServo = hardwareMap.get(Servo.class, "xServo");
        yServo = hardwareMap.get(Servo.class, "yServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");



        telemetry.addLine("clink clink grrrr");
        telemetry.update();
    }

    @Override
    public void loop() {
        xServo.setPosition(xPos);
        yServo.setPosition(yPos);
        rotateServo.setPosition(rotatePos);
        clawServo.setPosition(clawPos);

        telemetry.addData("xServo", xPos);
        telemetry.addData("yServo", yPos);
        telemetry.addData("rotateServo", rotatePos);
        telemetry.addData("clawServo", clawPos);
        telemetry.update();
    }
}
