package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.goBILDA_SWINGARM_POD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    //https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/

    private static double IN_TO_MM = 24.5; // 1 inch = 25.4 mm

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = goBILDA_SWINGARM_POD; // mm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -4 * IN_TO_MM; //mm; X is the up and down direction
    public static double PARALLEL_Y = 1 * (TRACK_WIDTH / 2) - 2.25 * IN_TO_MM; //mm; Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    private GoBildaPinpointDriver odo = null;// Declare OpMode member for the Odometry Computer

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(PARALLEL_X, PARALLEL_Y);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * IN_TO_MM * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

        return Arrays.asList(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        Pose2D vel = odo.getVelocity();
        return Arrays.asList(
                vel.getX(DistanceUnit.INCH),
                vel.getY(DistanceUnit.INCH)
        );
    }
}