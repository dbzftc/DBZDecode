package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "spmOpMode")
public class spmOpMode extends OpMode {

    RobotContainer robot;
    double powMult = 0.7;
    private int intakeStage = 0;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private boolean intakeInProgress = false;
    @Override
    public void init() {
        robot = new RobotContainer(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();


        telemetry.addLine("gurgle gurgle");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        intake();
        telemetry.update();

    }
    private void drive() {
        double drive = -gamepad1.left_stick_y;  // invert Y
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightPower = drive - turn;




        leftPower = Math.max(-1, Math.min(leftPower, 1));
        rightPower = Math.max(-1, Math.min(rightPower, 1));

        robot.hardware.frontLeft.setPower(leftPower * powMult);
        robot.hardware.backLeft.setPower(leftPower * powMult);
        robot.hardware.frontRight.setPower(rightPower * powMult);
        robot.hardware.backRight.setPower(rightPower * powMult);


        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);

    }
    private void intake() {
        if (gamepad1.a && !intakeInProgress) {
            intakeInProgress = true;
            intakeStage = 1;
            intakeTimer.reset();
        }

        if (!intakeInProgress) return;
        switch (intakeStage) {
            case 1:
                robot.hardware.xServo.setPosition(0.5);
                robot.hardware.yServo.setPosition(0.5);
                robot.hardware.rotateServo.setPosition(0.5);
                robot.hardware.clawServo.setPosition(0.5);

                intakeStage = 2;
                intakeTimer.reset();
                break;
            case 2:
                robot.hardware.intakeMotor.setPower(1);
                intakeStage = 3;
                intakeTimer.reset();
                break;
            case 3:
                robot.hardware.xServo.setPosition(0.5);
                intakeStage = 4;
                intakeTimer.reset();
                break;

            case 4:
                if(gamepad1.a) {
                    robot.hardware.yServo.setPosition(0.5);
                    robot.hardware.rotateServo.setPosition(0.5);
                    robot.hardware.clawServo.setPosition(0.5);
                    intakeStage = 5;
                    intakeTimer.reset();
                    }
                break;

            case 5:
                robot.hardware.intakeMotor.setPower(-1);
                intakeStage = 6;
                intakeTimer.reset();
                break;

            case 6:
                robot.hardware.xServo.setPosition(0.5);
                robot.hardware.yServo.setPosition(0.5);
                robot.hardware.rotateServo.setPosition(0.5);
                robot.hardware.clawServo.setPosition(0.5);
                intakeStage = 0;
                intakeTimer.reset();
                }
        }

    }
