package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Brandon on 10/3/2017.
 */

public class DbzCrServo implements CRServo,DbzDevice,PwmControl{

    private final CRServoImplEx crServoImplEx;

    public DbzCrServo(CRServoImplEx crServoImplEx){
        this.crServoImplEx = crServoImplEx;
    }

    public void init(Direction direction){
        //setPwmEnable();
        setDirection(direction);
        setPower(0);
    }

    @Override
    public ServoController getController() {
        return crServoImplEx.getController();
    }

    @Override
    public int getPortNumber() {
        return crServoImplEx.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        crServoImplEx.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return crServoImplEx.getDirection();
    }

    @Override
    public void setPower(double power) {
        crServoImplEx.setPower(power);
    }

    @Override
    public double getPower() {
        return crServoImplEx.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return crServoImplEx.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return crServoImplEx.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return crServoImplEx.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return crServoImplEx.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        crServoImplEx.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        crServoImplEx.close();
    }

    @Override
    public void setPwmRange(PwmRange range) {
        crServoImplEx.setPwmRange(range);
    }

    @Override
    public PwmRange getPwmRange() {
        return crServoImplEx.getPwmRange();
    }

    @Override
    public void setPwmEnable() {
        crServoImplEx.setPwmEnable();
    }

    @Override
    public void setPwmDisable() {
        crServoImplEx.setPwmDisable();
    }

    @Override
    public boolean isPwmEnabled() {
        return crServoImplEx.isPwmEnabled();
    }
}

