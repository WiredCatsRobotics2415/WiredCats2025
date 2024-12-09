package frc.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements;

/** An instance of a hotspot */
public class Hotspot {
    // Target coordinates
    private double x;
    private double y;
    private Pose2d targetPose;
    private Translation2d speakerLocation;
    private Translation2d targetCoords;

    /**
     * Make a new Hotspot.
     *
     * @param x: X coordinate (IN METERS)
     * @param y: Y coordinate (IN METERS)
     */
    public Hotspot(double x, double y) {
        this.x = x;
        this.y = y;
        this.targetCoords = new Translation2d(this.x, this.y);
        this.speakerLocation = Measurements.getSpeakerLocation();
    }

    /**
     * Calculate rotation to face a hotspot. Accounts for the fact that the shooter is on
     * the opposite side of the robot by rotating by 180.
     *
     * @param X: X coordinate of robot (IN METERS)
     * @param Y: Y coordinate of robot (IN METERS)
     * @return Rotation that the drivebase should be at to face the speaker.
     */
    public double rotEstimation(double X, double Y) {
        // TODO: speakerLocation used here is not going to update. Make sure to update it
        // here.
        Translation2d speakerDist = speakerLocation.minus(targetCoords);
        Rotation2d heading = Rotation2d
            .fromRadians(Math.atan2(speakerDist.getY(), speakerDist.getX()))
            .plus(Rotation2d.fromDegrees(180));
        return heading.getDegrees();
    }

    public double getX() { return this.x; }

    public double getY() { return this.y; }

    public Translation2d get2d() {
        return targetCoords;
    }

    /**
     * Constructs a target command using PathPlanner's Pathfinding. Uses the rotEstimation
     * method to get the target rotation. Uses PathPlanner defaults for max values.
     *
     * @return the pathfinding command. Can be called on SwerveDrive
     */
    public Command target() {
        // Create the target pose now so it has the updated speakerlocation
        this.targetPose = new Pose2d(this.x, this.y,
            Rotation2d.fromDegrees(rotEstimation(this.x, this.y)));
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should
        // travel before
        // attempting to rotate.
        );

        return pathfindingCommand;
    }
}
