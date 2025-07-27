package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.utils.LogDbz;

/**
 * Created by Matthew on 8/25/2017.
 */

public class DbzMotor implements DcMotorEx, DbzDevice {
    private static final String TAG = DbzMotor.class.getName();
    private final DcMotorEx dcMotorEx;

    private boolean isReversed = false;

    public DbzMotor(DcMotorEx dcMotorEx) {
        this.dcMotorEx = dcMotorEx;

        if (dcMotorEx instanceof DbzMotor)
            org.firstinspires.ftc.teamcode.LogDbz.w(TAG, "Someone just made a DbzMotor wrapper around another DbzMotor; this is probably not intended");
    }

    public void init(ZeroPowerBehavior zeroPowerBehavior, Direction direction) {
        setZeroPowerBehavior(zeroPowerBehavior);
        setDirection(direction);
        //So, when we don't do this, the motor oscillates between stopping when we set the velocity
        //between 0 and 1. We changed the p coefficient to 0 and the issue went away. We shouldn't have to do this
        //and it doesn't guarantee that the motor powers will work the same
        //PIDCoefficients pidCoefficients = getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //pidCoefficients.p = 0d;
        //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
    }

    public void setReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }

    public void resetEncoders(){
        this.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setMode(RunMode.RUN_USING_ENCODER);


    }


    public boolean isReversed() {
        return isReversed;
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (isReversed) {
            dcMotorEx.setVelocity(-angularRate, unit);
        } else {
            dcMotorEx.setVelocity(angularRate, unit);
        }
    }

    @Override
    public void setPower(double power) {
        if (isReversed) {
            dcMotorEx.setPower(-power);
        } else {
            dcMotorEx.setPower(power);
        }
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotorEx.setDirection(direction);
    }


    /* Methods dealing with velocity */

    /**
     * Get the maximum achievable radians per second
     *
     * @return the maximum number of radians per second the motor can spin at
     */
    public double getAchievableMaxRadiansPerSec() {
        MotorConfigurationType type = dcMotorEx.getMotorType();
        return 2 * Math.PI * type.getAchieveableMaxRPMFraction() * type.getMaxRPM() / 60;
    }

    public void setVelocityRadiansUsingPower(double radiansRate) {
        setPower(radiansRate / getAchievableMaxRadiansPerSec());
    }

    /* */
    public double getTicksPerRev() {
        return getMotorType().getTicksPerRev();
    }

    public boolean isBusyPhone() {
        return (Math.abs(getCurrentPosition() - getTargetPosition()) > getTargetPositionTolerance());
    }

    /* Delegate all other methods to DcMotorEx */

    /**
     * @param unit
     * @return
     */
    @Override
    public double getVelocity(AngleUnit unit) {
        return dcMotorEx.getVelocity(unit);
    }

    @Override
    public double getPower() {
        return dcMotorEx.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotorEx.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotorEx.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotorEx.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dcMotorEx.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotorEx.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dcMotorEx.close();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotorEx.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotorEx.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotorEx.getController();
    }

    @Override
    public Direction getDirection() {
        return dcMotorEx.getDirection();
    }

    @Override
    public int getPortNumber() {
        return dcMotorEx.getPortNumber();
    }

    @Override
    public boolean isBusy() {
        return dcMotorEx.isBusy();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotorEx.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        org.firstinspires.ftc.teamcode.LogDbz.w(TAG, "Call made to setPowerFloat, a deprecated method");
        dcMotorEx.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotorEx.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotorEx.setTargetPosition(position);

    }

    @Override
    public synchronized int getTargetPosition() {
        return dcMotorEx.getTargetPosition();
    }

    @Override
    public synchronized int getCurrentPosition() {
        return dcMotorEx.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotorEx.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotorEx.getMode();
    }

    @Override
    public void setMotorEnable() {
        dcMotorEx.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        dcMotorEx.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return dcMotorEx.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        dcMotorEx.setVelocity(angularRate);
    }

    @Override
    public double getVelocity() {
        return dcMotorEx.getVelocity();
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        dcMotorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        dcMotorEx.setPositionPIDFCoefficients(p);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        dcMotorEx.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return dcMotorEx.getPIDCoefficients(mode);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        dcMotorEx.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return dcMotorEx.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return dcMotorEx.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        dcMotorEx.setTargetPositionTolerance(tolerance);
    }
}

