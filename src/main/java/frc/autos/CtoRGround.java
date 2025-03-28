package frc.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.constants.Controls.Presets;
import frc.robot.Robot;

public class CtoRGround extends GenericAuto {
    private PathPlannerPath followPath;

    @Override
    public void initialize() {
        pathNames.add("CtoLeftGround");
        pathNames.add("LeftIntake");
        pathNames.add("LeftIntakeFinal");
        addAutos();

        // Set pos based on limelight and MT1, we may wanna have set pose starts instead
        // TODO: since you tested in simulator without these commands actually running (these are commands not functions) and it worked, I won't touch it
        drive.resetRotationFromLimelightMT1().schedule();
        drive.resetPoseFromLimelight().schedule();

        System.out.println("NEW RUN!!!");
        new AlignToReef(Side.Left).alongWith(new ReefPresetTo(Level.L4)).withTimeout(4)
            .andThen(new InstantCommand(() ->
            {
                // endEffector outtakes to let go of coral
                endEffector.toggleOuttakeCoral().andThen(Commands.waitSeconds(3)).andThen(endEffector.turnOff())
                    .schedule();
                System.out.println("Ended scoring coral. Now starting path to HPS.");
                followPath = paths.stream().filter(path -> path.name.equals("CtoLeftGround")).findFirst().orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
            })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new InstantCommand(() -> {
                // goes into knock over, and turns on end effector and coral intake
                superstructure.beThereAsapNoEnd(Presets.KnockOverStack).schedule();
                endEffector.toggleIntakeCoral().schedule();
                coralIntake.toggleIntake().schedule();
                System.out.println("intaking");
                followPath = paths.stream().filter(path -> path.name.equals("LeftIntake")).findFirst().orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
            })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new InstantCommand(() -> {
                // goes into finalIntake
                superstructure.beThereAsapNoEnd(Presets.GroundIntake).schedule();
                System.out.println("intaking final");
                followPath = paths.stream().filter(path -> path.name.equals("LeftIntakeFinal")).findFirst()
                    .orElse(null);
                System.out.println(followPath.name);
                AutoBuilder.followPath(followPath).schedule();
            })).andThen(new WaitUntilCommand(() -> atTarget)).andThen(new InstantCommand(() -> {
                // turns off coral intake, but not end effector, to keep the coral inside
                coralIntake.turnOffRollers().schedule();
            })).andThen(new AlignToReef(Side.Left).alongWith(new ReefPresetTo(Level.L3)).withTimeout(4)
                .andThen(new InstantCommand(() ->
                {
                    endEffector.toggleOuttakeCoral().withTimeout(3);
                    System.out.println("Auto is done!");
                }))).schedule();

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
