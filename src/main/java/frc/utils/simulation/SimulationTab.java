package frc.utils.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SimulationTab {
    private static SimulationTab instance;
    private ShuffleboardTab thisTab;

    private SimulationTab() {
        thisTab = Shuffleboard.getTab("Simulation");

        thisTab.add("Insert Coral at Left HPS", new InstantCommand(() -> {
            SimulatedArena.getInstance()
                .addGamePiece(new ReefscapeCoralOnField(new Pose2d(1.276731, 7.0, Rotation2d.kZero)));
        }).withName("Left HPS"));

        thisTab.add("Insert Coral at Right HPS", new InstantCommand(() -> {
            SimulatedArena.getInstance()
                .addGamePiece(new ReefscapeCoralOnField(new Pose2d(0.75, 0.75, Rotation2d.kZero)));
        }).withName("Right HPS"));
    }

    public static void enableSimulationControls() {
        if (instance == null) {
            instance = new SimulationTab();
        }
    }
}
