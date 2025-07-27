package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

/**
 * An extension of the normal hardwareMap that provides only Dbz-extended objects instead of just anything
 */

public class DbzHardwareMap {
    private static final String TAG = DbzHardwareMap.class.getName();

    private static HardwareMap hardwareMap;

    static {
        hardwareMap = OpMode.getInstanceHardwareMap();
    }

    /**
     * A map that maps the name of a device to the actual Dbz hardware device
     * When one of the get methods is called, we check this map to see if the Dbz device has already been created
     * If it has, we just return that object instead of making a new one
     */
    private static HashMap<DbzDeviceNames, DbzDevice> createdDevices = new HashMap<>();

    public static Context getAppContext() {
        return hardwareMap.appContext;
    }

    public static DbzMotor getDbzMotor(DbzMotorNames motor) {
        // if we've already created this DbzMotor, just return that DbzMotor
        if (createdDevices.containsKey(motor)) {
            if (createdDevices.get(motor) instanceof DbzMotor)
                return (DbzMotor) createdDevices.get(motor);
        }

        // Otherwise, go get the DcMotor out of hardwareMap
        DcMotor base = hardwareMap.get(DcMotor.class, motor.getName());

        // we can only make a DbzMotor if we got an DcMotorEx from hardwareMap
        // if we got one, then put it in createdDevices and return it.  otherwise be angry.
        if (base instanceof DcMotorEx) {
            DbzMotor dbzMotor = new DbzMotor((DcMotorEx) base);
            createdDevices.put(motor, dbzMotor);
            return dbzMotor;
        } else {
            throw new RuntimeException("Motor " + motor.getName() + " is not an Ex type; it is " + base.getClass().getSimpleName()
                    + " are you sure the attached hardware supports DcMotorEx?");
        }
    }

    public static DbzCrServo getDbzCRServo(DbzCrServoNames crServo) {
        if (createdDevices.containsKey(crServo)) {
            if (createdDevices.get(crServo) instanceof DbzCrServo)
                return (DbzCrServo) createdDevices.get(crServo);
        }
        CRServo base = hardwareMap.get(CRServo.class, crServo.getName());

        if (base instanceof CRServoImplEx) {
            DbzCrServo dbzCrServo = new DbzCrServo((CRServoImplEx) base);
            createdDevices.put(crServo, dbzCrServo);
            return dbzCrServo;
        } else {
            throw new RuntimeException("CRServo " + crServo.getName() + " is not an ImplEx type; it is " + base.getClass().getSimpleName()
                    + " are you sure the attached hardware supports CRServoImplEx?");
        }

    }

    public static DbzServo getDbzServo(DbzServoNames servo) {
        // if we've already created this DbzServo, just return that DbzServo
        if (createdDevices.containsKey(servo)) {
            if (createdDevices.get(servo) instanceof DbzServo)
                return (DbzServo) createdDevices.get(servo);
        }

        Servo base = hardwareMap.get(Servo.class, servo.getName());

        // we can only make a DbzServo if we got an ServoEx from hardwareMap
        // if we got one, then put it in createdDevices and return it.  otherwise be angry.
        if (base instanceof ServoImplEx) {
            DbzServo dbzServo = new DbzServo((ServoImplEx) base);
            createdDevices.put(servo, dbzServo);
            return dbzServo;
        } else {
            throw new RuntimeException("Servo " + servo.getName() + " is not an Ex type; it is " + base.getClass().getSimpleName()
                    + " are you sure the attached hardware supports ServoEx?");
        }
    }

    public static DbzImu getDbzIMU(DbzIMUNames IMU) {
        if (createdDevices.containsKey(IMU)) {
            if (createdDevices.get(IMU) instanceof DbzImu) {
                return (DbzImu) createdDevices.get(IMU);
            }
        }
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, IMU.getName());
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Imu";

        imu.initialize(parameters);

        DbzImu dbzImu = new DbzImu(imu);

        createdDevices.put(IMU, dbzImu);
        return dbzImu;
    }

    public enum DbzMotorNames implements DbzDeviceNames {

        left("LeftMotor"), right("RightMotor"),
        frontRight("rightFront"), frontLeft("leftFront"),
        backRight("rightRear"), backLeft("leftRear"), horiz("horizMotor"),
        //        airplaneMotor("airplaneMotor"),
        slideMotorL("slideMotorL"),
        slideMotorR("slideMotorR");
//        intakeMotor("intakeMotor"),
//        winchMotor("winchMotor");


        private String name;

        DbzMotorNames(String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public enum DbzServoNames implements DbzDeviceNames {;

        private String name;

        DbzServoNames(String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public enum DbzCrServoNames implements DbzDeviceNames {;
        // wrist("wristServo");
        //   stoneGripServo("gripServo");


        private String name;

        DbzCrServoNames(String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public enum DbzIMUNames implements DbzDeviceNames {
        internalIMU("internalIMU"),internalIMU2("internalIMU2");

        String name;

        DbzIMUNames(String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public interface DbzDeviceNames {
        String getName();
    }


}
