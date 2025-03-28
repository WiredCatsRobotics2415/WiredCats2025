package frc.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.robot.Robot;

public class L4 extends GenericAuto {
    private PathPlannerPath followPath;

    @Override
    public void initialize() {
        pathNames.add("L4Backup");
        addAutos();

        // Set pos based on limelight and MT1, we may wanna have set pose starts instead
        // TODO: since you tested in simulator without these commands actually running (these are commands not functions) and it worked, I won't touch it
        drive.resetRotationFromLimelightMT1().schedule();
        drive.resetPoseFromLimelight().schedule();

        System.out.println("NEW RUN!!!");
        new ReefPresetTo(Level.L4).andThen(endEffector.toggleIntakeCoral()).andThen(Commands.waitSeconds(3.5))
            .andThen(new AlignToReef(Side.Left)).andThen(Commands.waitSeconds(2)).andThen(new InstantCommand(() ->
            {
                endEffector.toggleOuttakeCoral().andThen(Commands.waitSeconds(4)).andThen(endEffector.turnOff())
                    .schedule();
                System.out.println("Ended scoring coral.");
            })).andThen(Commands.waitSeconds(3)).andThen(new InstantCommand(() -> {
                System.out.println("Ending auto.");
                followPath = paths.stream().filter(path -> path.name.equals("L4Backup")).findFirst().orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
                superstructure.stow();
            })).schedule();
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
