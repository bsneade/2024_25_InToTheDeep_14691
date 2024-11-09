package org.firstinspires.ftc.teamcode.mm14691;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Locale;

@TeleOp

public class ManualDrive extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private GoBildaPinpointDriver odo = null;// Declare OpMode member for the Odometry Computer
    private DcMotor armViper = null;
    private DcMotor armLift= null;

    //TODO SET THIS VALUE
    public static final int VIPER_LIMIT = 4500;
    public static final int VIPER_HOME = 0;
    public static final int VIPER_HOLD_POWER = 0;

    //Set TargetPositions TODO
    int TargetHeight = 4125;

    DigitalChannel digitalTouch;  // Digital channel Object

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "rearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rearRight");
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        armViper = hardwareMap.get(DcMotor.class, "armViper");

        // TODO need to change the offset here for gamerobot
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Set the directions for the motors
        //TODO may need to adjust these
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotorSimple.Direction.FORWARD);
        armViper.setDirection(DcMotorSimple.Direction.FORWARD);

        //This will make sure that each time you hit run it starts at 0
        armViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Command the robot to 0 to begin with so it doesn't move
        armViper.setTargetPosition(0);

        // get a reference to our touchSensor object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "armViperLimit");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");

        // Set up the telemetry
        telemetry.addData("ODO Status", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.addData("Arm Lift Status", "initialized");
        telemetry.addData("Viper starting position",  armViper.getCurrentPosition());
        telemetry.update();
        // keep track of the viper motor 'start' position so we can calc the end position correctly
        int viperStartPosition = armViper.getCurrentPosition();

        // Waiting for start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

//            Request an update from the Pinpoint odometry computer. This checks almost all outputs
//            from the device in a single I2C read.

            odo.update();

            //Check button state
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            //Do appropriate movement for button state
            //Up:1,0,0
            //Left:0,1,0
            //Down:-1,0,0
            //Right:0,-1,0
            double leftFrontPower  = axial - lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial + lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            //Send power to wheels
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

//            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

//            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ODO Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
//below is the arm code
            double armLiftPower = gamepad2.left_stick_y;
            double armViperPower = -gamepad2.right_stick_y;

            // Send telemetry message to indicate successful Encoder reset and current position
            telemetry.addData("Viper End Limit",  armViper.getCurrentPosition());
            telemetry.addData("Viper Current Position",armViper.getCurrentPosition());
            telemetry.update();

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
            //run to target position
            //Tell the motor than you want to put it in closed loop position control
            //TODO if gamepad2.right_bumper pressed --do run to position
            armViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Now in your code, if you want to send the lift to a preset position (example coding it to the A,B,X,Y buttons):
            armViper.setTargetPosition(TargetHeight);

            //To change the speed of the motor, it looks something like this: (You can adjust the 4000 down to say 500 and it will move much slower)
            //((DcMotorEx) LiftMotor).setVelocity(4000);

            armLift.setPower(armLiftPower);
            armViper.setPower(armViperPower);
            telemetry.addData("Arm Viper Current Position", armViper.getCurrentPosition());
            telemetry.addData("Arm Lift Power/ ArmViper Power ", "%4.2f, %4.2f", armLiftPower,armViperPower);
            telemetry.update();


        }
    }
}
