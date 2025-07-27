package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
@TeleOp(name="OpModeJS", group="TeleOp")

public class OpModeJS {
    private double powMult = 0.95;
    protected DbzMotor backLeft;
    protected DbzMotor backRight;
    protected DbzMotor frontRight;
    protected DbzMotor frontLeft;
    protected org.firstinspires.ftc.teamcode.extensions.DbzGamepad dbzGamepad1;
    protected org.firstinspires.ftc.teamcode.extensions.DbzGamepad dbzGamepad2;

    private void commonInit() {
    frontLeft = DbzHardwareMap.getDbzMotor(DbzHardwareMap.DbzMotorNames.frontLeft);
    frontLeft.init(DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);
    frontRight = DbzHardwareMap.getDbzMotor(DbzHardwareMap.DbzMotorNames.frontRight);
    frontRight.init(DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
    backLeft = DbzHardwareMap.getDbzMotor(DbzHardwareMap.DbzMotorNames.backLeft);
    backLeft.init(DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);
    backRight = DbzHardwareMap.getDbzMotor(DbzHardwareMap.DbzMotorNames.backRight);
    backRight.init(DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
}

protected void opInit() {

    }


    protected void opLoopHook() {

    }


    protected void opLoop() {
        drive2();
    }



    private void drive2() {

        // Gamepad input
        double leftStickX = dbzGamepad1.left_stick_x;
        double leftStickY = dbzGamepad1.left_stick_y; // Reverse y-axis
        double rightStickX  = dbzGamepad1.right_stick_x;

        // Mecanum drive calculations
        double frontLeftPower = (leftStickY + leftStickX + rightStickX);
        double frontRightPower = leftStickY - leftStickX - rightStickX;
        double backLeftPower = (leftStickY - leftStickX + rightStickX);
        double backRightPower = leftStickY + leftStickX - rightStickX;

        // Scale the powers to prevent motor power overflow
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower * powMult);
        frontRight.setPower(frontRightPower * powMult);
        backLeft.setPower(backLeftPower * powMult);
        backRight.setPower(backRightPower * powMult);
    }

    protected void opTeardown() {

    }
}
