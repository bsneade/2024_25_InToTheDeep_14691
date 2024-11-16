package org.firstinspires.ftc.teamcode.mm14691;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

public class MMAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.update();

        // Waiting for start

        waitForStart();
        //restarts runtime

        while (opModeIsActive()) {

//            Request an update from the Pinpoint odometry computer. This checks almost all outputs
//            from the device in a single I2C read.
            PoseVelocity2d poseVelocity2d = drive.updatePoseEstimate();

        }
    }
}
