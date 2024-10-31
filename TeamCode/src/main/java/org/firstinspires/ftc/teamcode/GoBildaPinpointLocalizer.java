package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GoBildaPinpointLocalizer implements Localizer {

    //TODO - Make sure these values match the measurements of the robot
    public static class Params {
        public double offset_x = -4 * DriveConstants.IN_TO_MM; //mm; X is the up and down direction
        public double offset_y = 1 * (TRACK_WIDTH / 2) - 2.25 * DriveConstants.IN_TO_MM; //mm; Y is the strafe direction
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;

    private Pose2D lastPosition;
    private Pose2D lastVelocity;
    private Double lastHeading;

    public GoBildaPinpointLocalizer(HardwareMap hardwareMap) {
        // TODO - make sure the device name for pinpoint controller is set correctly
        driver = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        if (driver == null) {
            throw new IllegalStateException("Could not find the odo driver, check config");
        }

        //TODO - set the direction and resolution according to your hardware per
        // the GoBilda instructions.
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        driver.setOffsets(PARAMS.offset_x, PARAMS.offset_y);
        driver.resetPosAndIMU();

        FlightRecorder.write("GOBILDA_PINPOINT_PARAMS", PARAMS);
    }

    @Override
    public Twist2dDual<Time> update() {
        //fetch the updates from the odometry driver
        driver.update();

        //read the current values
        Pose2D position = driver.getPosition();
        Pose2D velocity = driver.getVelocity();
        Double heading = driver.getHeading();
        double headingVelocity = driver.getHeadingVelocity();

        //FIXME - update the flight recorder
//        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        //check if we have run before and have our last values
        if (lastPosition == null || lastVelocity == null || lastHeading == null) {
            // any one of the 'last' values is not set, reset them all and return
            lastPosition = position;
            lastVelocity = velocity;
            lastHeading = heading;

            // assume we are at the origin
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // calculate our angle
        //FIXME: we may need to convert from radians to degrees
        double headingDelta = heading - lastHeading;
        DualNum<Time> angleDual =  new DualNum<>(new double[] {
                headingDelta,
                headingVelocity
        });

        // calculate our line
        // FIXME - we may need to use INCH instead
        Vector2dDual<Time> lineDual = new Vector2dDual<>(
                new DualNum<Time>(new double[] {
                        position.getY(DistanceUnit.MM) - lastPosition.getY(DistanceUnit.MM) - headingDelta,
                        velocity.getY(DistanceUnit.MM) - headingVelocity,
                }),
                new DualNum<Time>(new double[] {
                        position.getX(DistanceUnit.MM) - lastPosition.getX(DistanceUnit.MM) - headingDelta,
                        velocity.getX(DistanceUnit.MM) - headingVelocity,
                })
        );

        // set up our result
        Twist2dDual<Time> twist = new Twist2dDual<>(
                lineDual,
                angleDual
        );

        //update our 'last' values for the next run
        lastPosition = position;
        lastVelocity = velocity;
        lastHeading = heading;

        return twist;
    }

}

