package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Arrays;
import java.util.List;

public class GoBildaPinpointLocalizer extends TwoTrackingWheelLocalizer {
    private static final double IN_TO_MM = 24.5; // 1 inch = 25.4 mm

    public static final double OFFSET_X = -4 * IN_TO_MM; //mm; X is the up and down direction
    public static final double OFFSET_Y = 1 * (TRACK_WIDTH / 2) - 2.25 * IN_TO_MM; //mm; Y is the strafe direction

    public static final double PERPENDICULAR_X = 0;
    public static final double PERPENDICULAR_Y = 0;

    // These are set from running the LocalizationTest
    // See https://learnroadrunner.com/dead-wheels.html#tuning-two-wheel
    public static double X_MULTIPLIER = 0.9915; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.00544375; // Multiplier in the Y direction

    private GoBildaPinpointDriver odo = null;
    private MecanumDrive drive;

    public GoBildaPinpointLocalizer(HardwareMap hardwareMap, MecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(OFFSET_X, OFFSET_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(OFFSET_X, OFFSET_Y);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Hold on to our back reference to the drive
        this.drive = drive;
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        Pose2D pos = odo.getPosition();
        return Arrays.asList(
                pos.getX(DistanceUnit.INCH) * X_MULTIPLIER,
                pos.getY(DistanceUnit.INCH) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        Pose2D vel = odo.getVelocity();
        return Arrays.asList(
                vel.getX(DistanceUnit.INCH) * X_MULTIPLIER,
                vel.getY(DistanceUnit.INCH) * Y_MULTIPLIER
        );
    }

    @Override
    public double getHeading() {
        return odo.getHeading();
    }

    @Nullable
    @Override
    public Double getHeadingVelocity() {
        return odo.getHeadingVelocity();
    }

    @Override
    public void update() {
        odo.update();

        super.update();
    }
}

