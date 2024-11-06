package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.SerialNumber;

public class GoBildaMockMotorController extends LynxDcMotorController {

    private LynxModule lynxModule;

    public GoBildaMockMotorController(LynxModule module) throws RobotCoreException, InterruptedException {
        this(null, module);
    }

    public GoBildaMockMotorController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
        this.lynxModule = module;
    }

    public LynxModule getLynxModule() {
        return lynxModule;
    }
}
