package frc.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.CustomAutos;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.robot.Robot;

public class CtoRHPS extends CustomAutos {
    private PathPlannerPath followPath;

    @Override
    public void initialize() {
        pathNames.add("CtoRHPS");
        pathNames.add("RHPStoReef");
        addAutos();

        // Set pos based on limelight and MT1, we may wanna have set pose starts instead
        // TODO: since you tested in simulator without these commands actually running (these are commands not functions) and it worked, I won't touch it
        drive.resetPoseFromLimelight();
        drive.resetRotationFromLimelightMT1();

        System.out.println("NEW RUN!!!");
        new AlignToReef(Side.Left).alongWith(new ReefPresetTo(Level.L4)).withTimeout(4)
            .andThen(new InstantCommand(() ->
            {
                endEffector.toggleOuttake().andThen(Commands.waitSeconds(3)).andThen(endEffector.turnOff()).schedule();
                System.out.println("Ended scoring coral. Now starting path to HPS.");
                followPath = paths.stream().filter(path -> path.name.equals("CtoRHPS")).findFirst().orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
            })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new InstantCommand(() -> {
                // TODO: HPS preset
            })).andThen(new WaitUntilCommand(() -> hasCoral)).andThen(new InstantCommand(() -> {
                System.out.println("Ended first path. Now starting path to Reef.");
                followPath = paths.stream().filter(path -> path.name.equals("RHPStoReef")).findFirst().orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
            })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new AlignToReef(Side.Left)
                .alongWith(new ReefPresetTo(Level.L3)).withTimeout(4).andThen(new InstantCommand(() ->
                {
                    endEffector.toggleOuttake().withTimeout(3);
                    System.out.println("Auto is done!");
                }))).schedule();
        // })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new ScoreCoral(Side.Left, Level.L3)).withTimeout(4)
        // .andThen(() ->
        // {
        // endEffector.toggleOuttake().withTimeout(3);
        // System.out.println("Finished autonomous");

    }

    @Override
    public void execute() {
        super.execute();
        atTarget = isAtTarget(followPath);
        if (Robot.isSimulation()) {
            // TODO: add sim button
            hasCoral = true;
        } else {
            hasCoral = endEffector.hasCoral();
        }
        // if (followPath != null) System.out.println(atTarget + followPath.name);
    }
}
