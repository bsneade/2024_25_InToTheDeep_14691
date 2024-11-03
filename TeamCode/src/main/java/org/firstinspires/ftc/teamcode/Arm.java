package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class Arm extends LinearOpMode {

    private DcMotor armViper = null;
    private DcMotor armLift= null;

    //TODO SET THIS VALUE
    public static final double VIPER_LIMIT = 100;
    public static final int VIPER_HOLD_POWER = 0;

    DigitalChannel digitalTouch;  // Digital channel Object

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize motors
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        armViper = hardwareMap.get(DcMotor.class, "armViper");
        //setdirection for motors
        armLift.setDirection(DcMotorSimple.Direction.FORWARD);
        armViper.setDirection(DcMotorSimple.Direction.FORWARD);

        // get a reference to our touchSensor object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "armViperLimit");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");

        //add telemetry
        telemetry.addData("Arm Lift Status", "initialized");
        telemetry.update();

        // keep track of the viper motor 'start' position so we can calc the end position correctly
        int viperStartPosition = armViper.getCurrentPosition();

        // Waiting for start
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double armLiftPower = gamepad2.left_stick_y;
            double armViperPower = -gamepad2.right_stick_y;

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Viper End Limit",  armViper.getCurrentPosition());

            // End limit hit, set power to 0
            if (armViper.getCurrentPosition() - viperStartPosition >= VIPER_LIMIT) {
                armViperPower = VIPER_HOLD_POWER;
                telemetry.addData("Viper End Limit", "Activated");
            } else {
                telemetry.addData("Viper End Limit", "Inactive");
            }

            // Start limit hit, set power to 0
            if (digitalTouch.getState() == false && armViperPower < 0) {
                viperStartPosition = armViper.getCurrentPosition();
                telemetry.addData("Arm Lift Status", "calibrated");
                armViperPower = 0;
                telemetry.addData("Viper Start Limit", "Activated");
            } else {
                telemetry.addData("Viper Start Limit", "Inactive");
            }

            armLift.setPower(armLiftPower);
            armViper.setPower(armViperPower);
            telemetry.addData("Arm Viper Current Position", armViper.getCurrentPosition());
            telemetry.addData("Arm Lift Power/ ArmViper Power ", "%4.2f, %4.2f", armLiftPower,armViperPower);
            telemetry.update();
        }
    }
}