package frc.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.Dealgae;
import frc.commands.DealgaePresetTo;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.constants.Controls.Presets;
import frc.robot.Robot;

public class L4AndDealgae extends GenericAuto {
    private PathPlannerPath followPath;
    private Command auto = new InstantCommand(() -> {
        superstructure.stow().schedule();
        followPath = paths.stream().filter(path -> path.name.equals("L4Start")).findFirst().orElse(null);
        System.out.println(followPath.name);
        AutoBuilder.followPath(followPath).schedule();
        System.out.println("Started first path");
    }).andThen(new ReefPresetTo(Level.L4)).andThen(endEffector.toggleIntakeCoral()).andThen(Commands.waitSeconds(1.5))
        .andThen(new AlignToReef(Side.Left)).andThen(Commands.waitSeconds(1.5))
        .andThen(endEffector.toggleOuttakeCoral()).andThen(Commands.waitSeconds(1)).andThen(endEffector.turnOff())
        .andThen(new InstantCommand(() ->
        {
            System.out.println("Ended scoring coral.");
            followPath = paths.stream().filter(path -> path.name.equals("L4Backup")).findFirst().orElse(null);
            System.out.println(followPath.name);
            AutoBuilder.followPath(followPath).schedule();
            new DealgaePresetTo(false).schedule();
        })).andThen(Commands.waitSeconds(1.3)).andThen(endEffector.toggleIntakeAlgae())
        .andThen(new AlignToReef(Side.Center, Presets.DealgaeDriveOffset)).andThen(new Dealgae())
        .andThen(Commands.waitUntil(endEffector::algaeSensorTrigger)).andThen(() ->
        {
            followPath = paths.stream().filter(path -> path.name.equals("L4Backup")).findFirst().orElse(null);
            System.out.println(followPath.name);
            AutoBuilder.followPath(followPath).schedule();
            superstructure.stow().schedule();
            endEffector.turnOff().schedule();
        });

    @Override
    public void initialize() {
        setStartingPosition(centerStartingPosition);
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
        pathNames.add("L4Start");
        pathNames.add("L4Backup");
        addAutos();
    }
}
