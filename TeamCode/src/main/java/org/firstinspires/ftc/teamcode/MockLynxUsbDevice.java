package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.content.Context;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager;
import com.qualcomm.robotcore.util.SerialNumber;

public class MockLynxUsbDevice extends LynxUsbDeviceImpl {

    public MockLynxUsbDevice(SerialNumber serialNumber) {
        this(null, serialNumber, null, null);
    }

    /**
     * Use {@link #findOrCreateAndArm} instead
     *
     * @param context
     * @param serialNumber
     * @param manager
     * @param usbManager
     */
    protected MockLynxUsbDevice(Context context, SerialNumber serialNumber, @Nullable Manager manager, RobotUsbManager usbManager) {
        super(context, serialNumber, manager, usbManager);
    }

    @SuppressLint("MissingSuperCall")
    @Override
    protected void finishConstruction() {
        //do nothing
    }

}
