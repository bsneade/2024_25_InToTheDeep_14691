package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * The RoadRunner Encoder interface is sealed for some reason.  This makes extending RoadRunner or adding
 * new hardware difficult.  Instead we are going to fake the DcMotor input with this wrapper class.
 */
public class GoBildaPinpointDcMotorExWrapper implements DcMotorEx {

    public enum MountDirection { PARALLEL, PERPENDICULAR }

    private MountDirection mountDirection;
    private GoBildaPinpointDriver driver;
    private LynxModule lynxModule = new LynxModule(
            new MockLynxUsbDevice(SerialNumber.createFake()), 0, false, false);

    public GoBildaPinpointDcMotorExWrapper(GoBildaPinpointDriver driver, MountDirection mountDirection) {
        this.driver = driver;
        this.mountDirection = mountDirection;
    }

    /**
     * Finds the relevant method for the configured position.
     * @param name prefix of the method being called.  EX: 'get' when the method name is 'getX'
     * @param parameterTypes any parameters the method may have
     * @return
     */
    private Method getPositionalMethod(String name, Class<?>... parameterTypes) {
        Method resultMethod = null;
        String suffix = mountDirection == MountDirection.PARALLEL ? "X" : "Y";
        try {
            resultMethod = driver.getClass().getMethod(name + suffix, parameterTypes);
        } catch (NoSuchMethodException e) {
            //do nothing, we do a check later.
        }

        //Let's throw an error into RoadRunner with something meaningful so we can trace extra stuff to implements
        if (resultMethod == null) {
            throw new UnsupportedOperationException("Could not find method " + name +
                    " with mountDirection " + mountDirection +
                    " and " + parameterTypes.length + " parameters;" +
                    " Is RoadRunner calling an as-yet implemented method?");
        }

        return resultMethod;
    }

    @Override
    public void setMotorEnable() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setMotorDisable() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public boolean isMotorEnabled() {
        return false; // we aren't a motor, so return false for now
    }

    @Override
    public void setVelocity(double angularRate) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public double getVelocity() {
        driver.update();
        Method method = getPositionalMethod("getVel");
        try {
            return (double) method.invoke(driver);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        driver.update();
        Method method = getPositionalMethod("getVel", AngleUnit.class);
        try {
            return (double) method.invoke(driver, unit);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public int getTargetPositionTolerance() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public boolean isOverCurrent() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public MotorConfigurationType getMotorType() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public DcMotorController getController() {
        //RoadRunner is hardcoded to the LynxDCMotorController, so we have to deal with that
        LynxDcMotorController motorController = null;
        try {
            motorController = new GoBildaMockMotorController(this.lynxModule);
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
        return motorController;
    }

    @Override
    public int getPortNumber() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setPowerFloat() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public boolean getPowerFloat() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setTargetPosition(int position) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public int getTargetPosition() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public boolean isBusy() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public int getCurrentPosition() {
        driver.update();
        Method method = getPositionalMethod("getEncoder");
        try {
            return (int) method.invoke(driver);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setMode(RunMode mode) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public RunMode getMode() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void setDirection(Direction direction) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public Direction getDirection() {
        Method method = getPositionalMethod("getEncoderDirection");
        try {
            return Direction.values()[(int) method.invoke(driver)]; //NOTE: The ordinal for EncoderDirection needs to match (gross I know)
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setPower(double power) {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public double getPower() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public Manufacturer getManufacturer() {
        return driver.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return driver.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return driver.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return driver.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    @Override
    public void close() {
        throw new UnsupportedOperationException("Not implemented in the wrapper");
    }

    public LynxModule getLynxModule() {
        return lynxModule;
    }
}
