package frc.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.AllianceDependent;
import frc.utils.tuning.TuneableBoolean;
import java.io.IOException;
import java.util.ArrayList;
import org.json.simple.parser.ParseException;

public class GenericAuto extends Command {
    protected EndEffector endEffector = EndEffector.getInstance();
    protected CoralIntake coralIntake = CoralIntake.getInstance();
    protected SuperStructure superstructure = SuperStructure.getInstance();
    protected ArrayList<String> pathNames = new ArrayList<>();
    protected PathPlannerPath currentPath;
    protected CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    protected boolean atTarget;
    protected boolean hasCoral;
    protected ArrayList<PathPlannerPath> paths = new ArrayList<>();

    private final TuneableBoolean setStartingPositionSetting = new TuneableBoolean(true, "AutoSetStartingPosition");
    protected final AllianceDependent<Pose2d> centerStartingPosition = new AllianceDependent<Pose2d>(
        new Pose2d(7.319, 3.910, Rotation2d.k180deg), new Pose2d(10.44, 4.16, Rotation2d.kZero));

    protected void addAutos() {
        for (String name : pathNames) {
            try {
                currentPath = PathPlannerPath.fromPathFile(name);
            } catch (FileVersionException e) {
                currentPath = null;
                e.printStackTrace();
            } catch (IOException e) {
                currentPath = null;
                e.printStackTrace();
            } catch (ParseException e) {
                currentPath = null;
                e.printStackTrace();
            }
            paths.add(currentPath);
        }
    }

    protected boolean isAtTarget(PathPlannerPath path) {
        Pose2d current = drive.getState().Pose;
        if (path != null) {
            if (AllianceDependent.isCurrentlyBlue() != true) {
                path = path.flipPath();
            }

            Pose2d target = path.getPathPoses().get(path.getPathPoses().size() - 1);
            if (current.getTranslation().getDistance(target.getTranslation()) < 0.5) {
                return true;

            }
        }
        return false;
    }

    protected void setStartingPosition(AllianceDependent<Pose2d> startingPosition) {
        if (setStartingPositionSetting.get()) drive.resetPose(startingPosition.get());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("command ended");
    }
}
