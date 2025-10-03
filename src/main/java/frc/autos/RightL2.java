package frc.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.robot.Robot;

public class RightL2 extends GenericAuto {
    private PathPlannerPath followPath;
    private Command auto = new InstantCommand(() -> {
        followPath = paths.stream().filter(path -> path.name.equals("RightL2Start")).findFirst().orElse(null);
        System.out.println(followPath.name);
        AutoBuilder.followPath(followPath).schedule();
        System.out.println("Started first path");
    }).until(() -> atTarget).andThen(new ReefPresetTo(Level.L2Scoring)).andThen(Commands.waitSeconds(1.5))
        .andThen(new AlignToReef(Side.Left)).andThen(Commands.waitSeconds(1.5))
        .andThen(endEffector.toggleOuttakeCoral()).andThen(Commands.waitSeconds(1)).andThen(endEffector.turnOff())
        .andThen(new InstantCommand(() ->
        {
            System.out.println("Ended scoring coral.");
            followPath = paths.stream().filter(path -> path.name.equals("RightL2End")).findFirst().orElse(null);
            System.out.println(followPath.name);
            AutoBuilder.followPath(followPath).schedule();
            Commands.waitSeconds(1);
            superstructure.stow().schedule();
            System.out.println("Ending auto.");
        }));

    @Override
    public void initialize() {
        // Set pos based on limelight and MT1, we may wanna have set pose starts instead
        // TODO: since you tested in simulator without these commands actually running (these are commands not functions) and it worked, I won't touch it
        setStartingPosition(rightStartingPosition);
        auto.schedule();
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

    @Override
    public void addPaths() {
        pathNames.add("RightL2Start");
        pathNames.add("RightL2End");
        addAutos();
    }
}
