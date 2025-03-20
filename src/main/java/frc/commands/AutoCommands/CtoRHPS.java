package frc.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.commands.CustomAutos;
import frc.commands.ScoreCoral;
import frc.commands.ScoreCoral.Level;
import frc.commands.ScoreCoral.Side;

public class CtoRHPS extends CustomAutos {
    private PathPlannerPath followPath;

    @Override
    public void initialize() {
        pathNames.add("CtoRHPS");
        pathNames.add("RHPStoReef");
        addAutos();

        // Set pos based on limelight and MT1, we may wanna have set pose starts instead
        drive.resetPoseFromLimelight();
        drive.resetRotationFromLimelightMT1();

        // TODO: add in the align to coral command after Liam finishes making them different
        System.out.println("Started scoring coral.");
        new ScoreCoral(Side.Left, Level.L3).withTimeout(4).andThen(new InstantCommand(() -> {
            System.out.println("Ended scoring coral. Now starting path to HPS.");
            followPath = paths.stream().filter(path -> path.name.equals("CtoRHPS")).findFirst().orElse(null);
            AutoBuilder.followPath(followPath).schedule();
        })).until(() -> isAtTarget(followPath)).andThen(new InstantCommand(() -> {
            System.out.println("Ended path to HPS. Now starting new path to reef.");
            followPath = paths.stream().filter(path -> path.name.equals("CtoRHPS")).findFirst().orElse(null);
            AutoBuilder.followPath(followPath).schedule();
        })).until(() -> isAtTarget(followPath)).andThen(new InstantCommand(() -> {
            System.out.println("Ended path to HPS. Starting ScoreCoral.");
            new ScoreCoral(Side.Left, Level.L3).withTimeout(4).schedule();
        })).schedule();
        // AutoBuilder.followPath(paths.stream().filter(path -> path.name.equals("CtoRHPS")).findFirst().orElse(null))
        // .schedule();
    }
}
