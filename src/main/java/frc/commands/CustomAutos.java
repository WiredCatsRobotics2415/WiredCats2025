package frc.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commands.GenericAutomation.AutomationMode;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import java.io.IOException;
import java.util.ArrayList;
import org.json.simple.parser.ParseException;

public class CustomAutos extends Command {
    protected EndEffector endEffector = EndEffector.getInstance();
    protected ArrayList<String> pathNames = new ArrayList<>();
    protected PathPlannerPath currentPath;
    protected CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    protected ArrayList<PathPlannerPath> paths = new ArrayList<>();

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
        GenericAutomation.setCurrentAutomationMode(AutomationMode.PresetAndAlign);
    }

    protected boolean isAtTarget(PathPlannerPath path) {
        Pose2d current = drive.getState().Pose;
        if (path != null) {
            Pose2d target = path.getPathPoses().get(path.getPathPoses().size() - 1);
            if (current.getTranslation().getDistance(target.getTranslation()) < 0.3) {
                System.out.println("was close enough");
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("command ended");
    }
}
