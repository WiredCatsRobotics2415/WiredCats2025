package frc.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.Measurements;
import frc.constants.TunerConstants;

public class TargetHotspot extends Command {
    // GENERAL
    private Command currentPathfindingCommand;

    public TargetHotspot() {
        addRequirements(TunerConstants.DriveTrain); // TODO: add other subsystems
    }

    @Override
    public void initialize() {
        Translation2d currentPose = TunerConstants.DriveTrain.getState().Pose
            .getTranslation();
        // System.out.println(currentPose);
        Hotspot closestHotspot = Measurements.Hotspots.get(0);
        double minDistance = closestHotspot.get2d().getDistance(currentPose);
        for (Hotspot hotspot : Measurements.Hotspots) {
            double distance = currentPose.getDistance(hotspot.get2d());
            if (distance < minDistance) {
                closestHotspot = hotspot;
                minDistance = distance;
            }
        }
        // System.out.println(closestHotspot.get2d());
        // currentPose = TunerConstants.DriveTrain.getRobotPose().getTranslation();
        currentPathfindingCommand = closestHotspot.target();
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(currentPathfindingCommand);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("done pathfinding");
        currentPathfindingCommand.end(true);
    }

    @Override
    public boolean isFinished() { return currentPathfindingCommand.isFinished(); }
}
