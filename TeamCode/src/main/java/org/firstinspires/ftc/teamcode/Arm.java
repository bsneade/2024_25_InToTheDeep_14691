package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Arm extends LinearOpMode {

    private DcMotor armViper = null;
    private DcMotor armLift= null;

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

        // Waiting for start
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double armLiftPower = gamepad2.left_stick_y;
            double armViperPower = gamepad2.right_stick_y;

            // button is pressed set power =0

            if (digitalTouch.getState() == true) {
                armViperPower = 0;
                telemetry.addData("Viper Limit", "Activated");
            } else {
                telemetry.addData("Viper Limit", "Inactive");
            }

            armLift.setPower(armLiftPower);
            armViper.setPower(armViperPower);
            telemetry.addData("Arm Lift Power/ ArmViper Power ", "%4.2f, %4.2f", armLiftPower,armViperPower);
            telemetry.update();
        }
    }
}