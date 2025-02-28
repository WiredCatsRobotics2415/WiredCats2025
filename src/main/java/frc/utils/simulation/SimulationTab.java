package frc.utils.simulation;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SimulationTab {
    private static SimulationTab instance;
    private ShuffleboardTab thisTab;

    private Random random = new Random();

    private final Translation2d leftStationDropCenter = new Translation2d(1.2, 6.6);
    private final Translation2d leftStationCloseEnd = new Translation2d(0.268, 5.754);
    private final Translation2d leftStationFarEnd = new Translation2d(2.626, 7.773);

    private final Translation2d rightStationDropCenter = new Translation2d(1.3, 1.111);
    private final Translation2d rightStationCloseEnd = new Translation2d(2.735, 0.284);
    private final Translation2d rightStationFarEnd = new Translation2d(0.270, 2.481);

    private Translation2d getRandomPositionInSemicircle(Distance width, Distance radius) {
        double angle = Math.toRadians(random.nextDouble() * 180);
        double r = Math.sqrt(random.nextDouble()) * radius.in(Meters);

        double x = r * Math.cos(angle); // Convert polar to Cartesian
        double y = r * Math.sin(angle);

        // Ensure x is within width
        if (x > width.in(Meters) / 2) x = width.in(Meters) / 2;
        if (x < -width.in(Meters) / 2) x = -width.in(Meters) / 2;

        return new Translation2d(x, y);
    }

    private SimulationTab() {
        thisTab = Shuffleboard.getTab("Simulation");

        // https://www.desmos.com/calculator/44o3ymgzv5
        // TODO: just math.random * 3 ft at random angle
        thisTab.add("Drop Coral at Left HPS", new InstantCommand(() -> {
            Translation2d drop = getRandomPositionInSemicircle(
                Meters.of(leftStationFarEnd.getDistance(leftStationCloseEnd)), Feet.of(4));
            Translation2d dropInField = drop.plus(leftStationDropCenter).rotateAround(leftStationDropCenter,
                Rotation2d.fromDegrees(-126));
            SimulatedArena.getInstance()
                .addGamePiece(new ReefscapeCoralOnField(new Pose2d(dropInField, Rotation2d.kZero)));
        }).withName("Left HPS").ignoringDisable(true));

        thisTab.add("Drop Coral at Right HPS", new InstantCommand(() -> {
            Translation2d drop = getRandomPositionInSemicircle(
                Meters.of(rightStationFarEnd.getDistance(rightStationCloseEnd)), Feet.of(4));
            Translation2d dropInField = drop.plus(rightStationDropCenter).rotateAround(rightStationDropCenter,
                Rotation2d.fromDegrees(-36));
            SimulatedArena.getInstance()
                .addGamePiece(new ReefscapeCoralOnField(new Pose2d(dropInField, Rotation2d.kZero)));
        }).withName("Right HPS").ignoringDisable(true));
    }

    public static void enableSimulationControls() {
        if (instance == null) {
            instance = new SimulationTab();
        }
    }
}
