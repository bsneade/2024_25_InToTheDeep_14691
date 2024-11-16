package org.firstinspires.ftc.teamcode.mm14691;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

public class MMAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.update();

        // Configure our paths
        Action action = drive.actionBuilder(new Pose2d(-47, -46, 0))
                .lineToX(94-47).build();

        // Waiting for start
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        runBlocking(action);

    }
}
