package frc.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Measurements.RobotMeasurements;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;
import lombok.Getter;
import lombok.Setter;

public class Dealgae extends Command {
    public static enum DealgaeAutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Setter @Getter private static DealgaeAutomationMode currentAutomationMode = DealgaeAutomationMode.PresetAndAlign;

    private static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);
    private static final Transform2d Offset = new Transform2d(CenterToBumper, Inches.of(0), new Rotation2d());
    private static final double DriveToleranceMeters = Units.inchesToMeters(2);

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();

    private Command driveCommand;
    private Command superStructureCommand;

    private Pose2d findNearestReefSideApriltag() {
        Pose2d currentPosition = CommandSwerveDrivetrain.getInstance().getState().Pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return currentPosition.nearest(ReefMeasurements.reefBlueApriltags);
        } else {
            return currentPosition.nearest(ReefMeasurements.reefRedApriltags);
        }
    }

    private boolean algaeOnTopForPose(Pose2d apriltagPose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefBlueApriltags.indexOf(apriltagPose));
        } else {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefRedApriltags.indexOf(apriltagPose));
        }
    }

    public Dealgae() {
        // TODO: Note that if the robot is not driveable while the preset is going even though
        // You are in the PresetOnly mode, consider moving this addRequirements line to
        // initialize so it doesn't require drive too early
        addRequirements(drive, Elevator.getInstance(), Arm.getInstance());
    }

    @Override
    public void initialize() {
        Pose2d apriltagPose = findNearestReefSideApriltag();
        boolean algaeOnTop = algaeOnTopForPose(apriltagPose);

        superStructureCommand = superStructure.runToPositionCommand(
            algaeOnTop ? Presets.TopAlgaeDescoreHeight : Presets.BottomAlgaeDescoreHeight,
            algaeOnTop ? Presets.TopAlgaeDescoreAngle : Presets.BottomAlgaeDescoreAngle);
        superStructureCommand.schedule();

        if (currentAutomationMode == DealgaeAutomationMode.PresetAndAlign) {
            driveCommand = drive.driveTo(apriltagPose.plus(Offset), DriveToleranceMeters);
            driveCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == DealgaeAutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }
}
