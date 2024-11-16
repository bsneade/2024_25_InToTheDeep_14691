package org.firstinspires.ftc.teamcode.mm14691;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Locale;

@TeleOp

public class ManualDrive extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    // Define class members
    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null; //the wrist servo
    private DcMotor ascend = null;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    //TODO NEED TO CHANGE TO OUR SETTINGS
    // * Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armViper = null;
    private DcMotor armLift= null;

    //TODO SET THIS VALUE
   public static final int VIPER_HOME = 0;
    public static final int VIPER_HOLD_POWER = 0;

    //Set TargetPositions TODO
    int TargetHeight = 4125;

    DigitalChannel digitalTouch;  // Digital channel Object

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our drive
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addData("Drive Status", "Initialized");

        // Initialize the arms motors
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        armViper = hardwareMap.get(DcMotor.class, "armViper");

        //set power behavior
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setDirection(DcMotorSimple.Direction.REVERSE);
        armViper.setDirection(DcMotorSimple.Direction.FORWARD);
        ascend = hardwareMap.get(DcMotor.class, "ascend");
        ascend.setDirection(DcMotorSimple.Direction.FORWARD);

        //This will make sure that each time you hit run it starts at 0
        armViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Command the robot to 0 to begin with so it doesn't move
//        armViper.setTargetPosition(0);


        // get a reference to our touchSensor object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "armViperLimit");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Arm Lift Status", "initialized");
        // keep track of the viper motor 'start' position so we can calc the end position correctly
        int viperStartPosition = armViper.getCurrentPosition();
        telemetry.addData("Viper Position",  viperStartPosition);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        //Update the screen
        telemetry.update();

        // Waiting for start
        waitForStart();

        //restarts runtime
        runtime.reset();

        while (opModeIsActive()){

            // Request an update from the Pinpoint odometry computer. This checks almost all outputs
            // from the device in a single I2C read.
            drive.updatePoseEstimate();

            /// GAME PAD
            //Check button state
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            //calculation of power to motor
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
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
            //sets power to motors
            drive.leftFront.setPower(leftFrontPower);
            drive.rightFront.setPower(rightFrontPower);
            drive.leftBack.setPower(leftBackPower);
            drive.rightBack.setPower(rightBackPower);

            // gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
            //adds telemetry updates when driving
            Pose2D pos = drive.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            // gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            Pose2D vel = drive.pinpoint.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            //for the joystick pressed, moves the
            double armLiftPower = gamepad2.left_stick_y;
            double armViperPower = -gamepad2.right_stick_y;

            // Send telemetry message to indicate successful Encoder reset and current position
            telemetry.addData("Viper End Limit",  armViper.getCurrentPosition());
            telemetry.addData("Viper Current Position",armViper.getCurrentPosition());
            //((DcMotorEx) LiftMotor).setVelocity(4000);
            //tells about current power of arm
            armLift.setPower(armLiftPower);

//            // End limit hit, set power to 0
//            if (armViper.getCurrentPosition() - viperStartPosition >= MecanumDrive.PARAMS.VIPER_LIMIT) {
//                armViperPower = VIPER_HOLD_POWER;
//                telemetry.addData("Viper End Limit", "Activated");
//            } else {
//                telemetry.addData("Viper End Limit", "Inactive");
//            }
//
//            // Start limit hit, set power to 0
//            if (digitalTouch.getState() == false && armViperPower < 0) {
//                viperStartPosition = armViper.getCurrentPosition();
//                telemetry.addData("Arm Lift Status", "calibrated");
//                armViperPower = 0;
//                telemetry.addData("Viper Start Limit", "Activated");
//            } else {
//                telemetry.addData("Viper Start Limit", "Inactive");
//            }
            armViper.setPower(armViperPower);

            //To change the speed of the motor, it looks something like this: (You can adjust the 4000 down to say 500 and it will move much slower)
            //adds current position of arm
            telemetry.addData("Arm Viper Current Position", armViper.getCurrentPosition());
            telemetry.addData("Arm Lift Power/ ArmViper Power ", "%4.2f, %4.2f", armLiftPower,armViperPower);
            // starts intake configuration
            // intake collection
            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            if(gamepad2.right_bumper){
                /* This is the intaking/collecting arm position */
//            armPosition = ARM_COLLECT;


                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
//            armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            else if (gamepad2.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
//            armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
//            armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
//            armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            // updates telemetry after everything
            telemetry.update();
            //when dpad is not press, no power for ascend
            boolean ascendUp=gamepad2.dpad_up;
            boolean ascendDown=gamepad2.dpad_down;
            if (ascendUp == true){
                ascend.setPower(1.0);
            }
            if(ascendDown == true){
                ascend.setPower(-1.0);
            }
            else {
                ascend.setPower(0);
            }


        }
    }
}
