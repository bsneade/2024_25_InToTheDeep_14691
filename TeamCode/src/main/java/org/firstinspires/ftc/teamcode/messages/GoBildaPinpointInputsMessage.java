package org.firstinspires.ftc.teamcode.messages;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GoBildaPinpointInputsMessage {

    public long timestamp;

    public double position_x;
    public double position_y;
    public double position_heading;
    public double velocity_x;
    public double velocity_y;
    public double velocity_heading;
    public double heading;
    public double headingVelocity;

    public GoBildaPinpointInputsMessage(Pose2D position, Pose2D velocity, Double heading, double headingVelocity) {
        this.timestamp = System.nanoTime();

        this.position_x = position.getX(DistanceUnit.INCH);
        this.position_y = position.getY(DistanceUnit.INCH);
        this.position_heading = position.getHeading(AngleUnit.DEGREES);
        this.velocity_x = velocity.getX(DistanceUnit.INCH);
        this.velocity_y = velocity.getY(DistanceUnit.INCH);
        this.velocity_heading = velocity.getHeading(AngleUnit.DEGREES);
        this.heading = heading;
        this.headingVelocity = headingVelocity;
    }
}
