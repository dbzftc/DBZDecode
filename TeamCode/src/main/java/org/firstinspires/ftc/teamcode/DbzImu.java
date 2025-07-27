package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.utils.LogDbz;

import java.util.Locale;

/**
 * Created by Matthew on 8/28/2017.
 */

public class DbzImu implements BNO055IMU, DbzDevice {
    private static final String TAG = DbzImu.class.getName();

    private final BNO055IMU imu;
    private Orientation angles;
    private double pastHeadingAngle;
    private double curHeadingAngle;
    private double deltaHeadingAngle;
    private boolean headingClockwise;
    private double largeDeltaHeading;
    private double largeDeltaFromAbsolute;
    private Parameters parameters = new Parameters();

    public DbzImu(BNO055IMU imu) {
        this.imu = imu;
        largeDeltaHeading = 0;
        largeDeltaFromAbsolute = 0;
    }

    public boolean isBroken(int attempts){
        for (int i = 0; i < attempts; i++) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                org.firstinspires.ftc.teamcode.LogDbz.e(TAG,"interrupted while checking to see if IMU was broken", e);
            }
            if (getHeadingDegrees() != 0.0)
                return false;
        }
        return true;
    }

    public boolean isBroken(){
        return isBroken(10);
    }

    public Parameters getParameters() {
        return imu.getParameters();
    }

    public boolean initialize(Parameters parameters) {
        return imu.initialize(parameters);
    }

    public SystemStatus getSystemStatus() {
        return imu.getSystemStatus();
    }

    public SystemError getSystemError() {
        return imu.getSystemError();
    }

    public CalibrationStatus getCalibrationStatus() {
        return imu.getCalibrationStatus();
    }

    public void close() {
        imu.close();
    }

    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return imu.getAngularOrientation(reference, order, angleUnit);
    }

    public boolean isSystemCalibrated() {
        return imu.isSystemCalibrated();
    }

    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    public boolean isAccelerometerCalibrated() {
        return imu.isAccelerometerCalibrated();
    }

    public boolean isMagnetometerCalibrated() {
        return imu.isMagnetometerCalibrated();
    }

    public CalibrationData readCalibrationData() {
        return imu.readCalibrationData();
    }

    public void writeCalibrationData(CalibrationData data) {
        imu.writeCalibrationData(data);
    }

    public Temperature getTemperature() {
        return imu.getTemperature();
    }

    public MagneticFlux getMagneticFieldStrength() {
        return imu.getMagneticFieldStrength();
    }

    public Acceleration getOverallAcceleration() {
        return imu.getOverallAcceleration();
    }

    public Acceleration getLinearAcceleration() {
        return imu.getLinearAcceleration();
    }

    public Acceleration getGravity() {
        return imu.getGravity();
    }

    public AngularVelocity getAngularVelocity() {
        return imu.getAngularVelocity();
    }

    public Orientation getAngularOrientation() {
        return imu.getAngularOrientation();
    }

    public Quaternion getQuaternionOrientation() {
        return imu.getQuaternionOrientation();
    }

    public Acceleration getAcceleration() {
        return imu.getAcceleration();
    }

    public Velocity getVelocity() {
        return imu.getVelocity();
    }

    public Position getPosition() {
        return imu.getPosition();
    }

    public void startAccelerationIntegration(Position initalPosition, Velocity initialVelocity, int msPollInterval) {
        imu.startAccelerationIntegration(initalPosition, initialVelocity, msPollInterval);
    }

    public void stopAccelerationIntegration() {
        imu.stopAccelerationIntegration();
    }

    public byte read8(Register reg) {
        return imu.read8(reg);
    }

    public byte[] read(Register reg, int cb) {
        return imu.read(reg, cb);
    }

    public void write8(Register reg, int data) {
        imu.write8(reg, data);
    }

    public void write(Register reg, byte[] data) {
        imu.write(reg, data);
    }

    private String formatAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit, double angle) {
        return formatRadians(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }

    private String formatRadians(double radians) {
        radians = toPositiveRadians(radians);
        return String.format(Locale.getDefault(), "%.1f", (radians));
    }

    private double toPositiveRadians(double radians) {
        if (radians < 0) {
            radians += (2 * Math.PI);
        }
        return radians;
    }

    public void update() {
        angles = getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        //updateLargeDeltaHeading();
    }

    public String getHeadingString() {
        return formatAngle(angles.angleUnit, (angles.firstAngle));
    }

    public String getRollString() {
        return formatAngle(angles.angleUnit, (angles.secondAngle));
    }

    public String getPitchString() {
        return formatAngle(angles.angleUnit, (angles.thirdAngle));
    }

    public double getHeadingDouble() {
        return toPositiveRadians(angles.firstAngle);
    }
    public double getHeadingDegrees(){
        return (toPositiveRadians(angles.firstAngle) * 360) / (2 * Math.PI);
    }

    public double getRollDouble() {
        return toPositiveRadians(angles.secondAngle);
    }

    public double getPitchDouble() {
        return toPositiveRadians(angles.thirdAngle);
    }

    private double initialHeading = 0;

    /**This method along with startCollectingDeltaHeading() is used to calculate the delta angle from
     * any starting angle. This method takes the wraparound into the calculation of the largeDeltaHeading.
     * The largeDeltaHeading indicates the direction. (+ for CW) (- for CCW)
     * @return largeDeltaHeading - the delta angle from when the startCollectingDeltaHeading() is called
     * Note:The size of the deltas matter on the loop iteration time. This may affect the timing of receiving the largeDeltaTime
     */
    public double updateLargeDeltaHeading() {
        headingClockwise = false;
        curHeadingAngle = getHeadingDegrees();
        if (Math.abs(curHeadingAngle - pastHeadingAngle) > Math.PI) {//300 is an arbitrary number. In reality, the iteration of the loop should cause the delta
            //angle to be small, and the only time when it would not be, is if the angle is changing from 360 to 0 or vise versa. This
            //will calculate the delta angle during the wraparound of the heading angle
            if (curHeadingAngle > pastHeadingAngle) {//Turning CCW
                headingClockwise = false;
                curHeadingAngle -= 2*Math.PI;
                deltaHeadingAngle = Math.abs(curHeadingAngle - pastHeadingAngle);
            } else {//Turning CW
                headingClockwise = true;
                pastHeadingAngle -= 2*Math.PI;
                deltaHeadingAngle = Math.abs(curHeadingAngle - pastHeadingAngle);
            }
        }
        //If the angles are not in wraparound, simply calculate the delta
        else {
            if (curHeadingAngle > pastHeadingAngle) {
                headingClockwise = true;
                deltaHeadingAngle = Math.abs(curHeadingAngle - pastHeadingAngle);
            } else {
                headingClockwise = false;
                deltaHeadingAngle = Math.abs(curHeadingAngle - pastHeadingAngle);
            }
        }
        //deltaHeadingAngle = Math.round(deltaHeadingAngle * 1000) / 1000;
        if (!headingClockwise)
            deltaHeadingAngle = Math.abs(deltaHeadingAngle);
        else
            deltaHeadingAngle = -Math.abs(deltaHeadingAngle);

        largeDeltaFromAbsolute += deltaHeadingAngle;
        if (largeDeltaFromAbsolute < -(Math.PI * 2)){
            largeDeltaFromAbsolute += (Math.PI * 2);
        }
        else if (largeDeltaFromAbsolute > (Math.PI * 2)) {
            largeDeltaFromAbsolute -= (Math.PI * 2);
        }
        largeDeltaHeading += deltaHeadingAngle;//Add the small deltas to the large delta
        pastHeadingAngle = curHeadingAngle;

        return largeDeltaHeading;
    }

    private boolean startCollectingDeltaHeading = false;
    /**
     * This method should be called when you want to begin to check the largeDeltaHeading.
     * The initial angle will default to 0 if implemented properly
     * Call setInitialHeading if you want to change the initial heading of where the delta heading
     * begins to be calculated
     */
    public void startCollectingDeltaHeading() {
        startCollectingDeltaHeading = true;
        largeDeltaHeading = initialHeading;
        pastHeadingAngle = getHeadingDegrees();
    }

    /**
     * This method should be called when you finished checking the largeDeltaHeading
     * This is so that the largeDeltaHeading number does not become extremely large, and it might
     * save computing time if the largeDeltaHeading is not always being updated
     *
     */
    public void stopCollectingDeltaHeading() {
        startCollectingDeltaHeading = false;
        initialHeading = 0;
        deltaHeadingAngle = 0;
        curHeadingAngle = 0;
        pastHeadingAngle = 0;
    }

    public double getLargeDeltaHeading() {
        return Math.abs(largeDeltaHeading);
    }

    public double getSignedLargeDeltaHeading() {
        return largeDeltaHeading;
    }

    /**
     * Sets the initial heading to where the delta angle starts to calculate
     * @param heading - Radians
     */
    public void setIntialHeading(double heading) {
        initialHeading = heading;
    }

    /**
     * Set the reference angle to be tracked from the current angle
     *
     */
    public void setReferenceAngle() {
        largeDeltaFromAbsolute = 0;
    }

    /**
     * Indicates the direction of where the reference angle is from the current
     * @return the angle from the reference angle where setReferenceAngle() was called
     */
    public double getAngleFromReferenceAngle() {
        return largeDeltaFromAbsolute;
    }

    public boolean isDeltaPositive() {
        return (deltaHeadingAngle > 0);
    }
    /**
     * This method calculates the reference angle in the first quadrant from any given angle
     * @param angle - In radians
     * @return The angle in the first quadrant
     */
    public double angleInFirstQuadrant(double angle) {
        double angleInFirstQuadrant = 0;
        if (angle >= 0 && angle <= (Math.PI / 2)){
            angleInFirstQuadrant = angle;
        }
        if (angle > (Math.PI / 2)  && angle <= (Math.PI)){
            angleInFirstQuadrant = Math.PI - angle;
        }
        else if (angle > Math.PI && angle <= ((3 * Math.PI) / 2)){
            angleInFirstQuadrant = angle - Math.PI;
        }
        else if (angle > (3 * Math.PI) / 2 && angle <= (2 * Math.PI)){
            angleInFirstQuadrant = (2 * Math.PI) - angle;
        }
        else if (angle > (2 * Math.PI)){
            angleInFirstQuadrant = angle;
            while(angleInFirstQuadrant > (2 * Math.PI)){
                angleInFirstQuadrant -= (Math.PI * 2);
            }
            angleInFirstQuadrant = angleInFirstQuadrant(angleInFirstQuadrant);
        }
        else if (angle < 0) {
            angleInFirstQuadrant = angle;
            while(angleInFirstQuadrant < 0){
                angleInFirstQuadrant -= (2 * Math.PI);
            }
            angleInFirstQuadrant = angleInFirstQuadrant(angleInFirstQuadrant);
        }
        return angleInFirstQuadrant;
    }

    private double simplifyAngle(double angle) {
        double finalAngle;
        finalAngle = Math.abs(angle);
        while (angle > (Math.PI * 2)) {
            finalAngle -= (2 * Math.PI);
        }
        if (finalAngle > Math.PI) {
            finalAngle = (2 * Math.PI) - finalAngle;
        }
        return finalAngle;
    }

    public void resetImu() {
        imu.initialize(parameters);
    }
}

