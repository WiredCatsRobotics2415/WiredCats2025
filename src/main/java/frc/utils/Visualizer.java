package frc.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.constants.Subsystems.ElevatorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;
import frc.utils.tuning.TuneableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Reads from the elevator and publishes pose to NT for advantagescope
 */
public class Visualizer {

    private static Elevator elevatorSubsystem = Elevator.getInstance();
    private static Arm armSubsystem = Arm.getInstance();

    public static void update() {
        // Elevator
        double height = elevatorSubsystem.getGoal().in(Inches);
        Pose3d elevatorBase = new Pose3d(0, 0, 0, Rotation3d.kZero);
        Pose3d elevatorStage2 = new Pose3d(0, 0,
            Units.inchesToMeters((height / ElevatorConstants.MaxHeightInches) * (ElevatorConstants.Stage2Height)),
            Rotation3d.kZero);
        Pose3d elevatorStage3 = new Pose3d(0, 0, Units.inchesToMeters((height / ElevatorConstants.MaxHeightInches) *
            (ElevatorConstants.Stage2Height + ElevatorConstants.Stage3Height)), Rotation3d.kZero);
        Pose3d carriage = new Pose3d(0, 0, Units.inchesToMeters(height), Rotation3d.kZero);

        Logger.recordOutput("Visualization/ElevatorBase", elevatorBase);
        Logger.recordOutput("Visualization/ElevatorStage2", elevatorStage2);
        Logger.recordOutput("Visualization/ElevatorStage3", elevatorStage3);
        Logger.recordOutput("Visualization/Carriage", carriage);

        // Arm
        double angleRads = armSubsystem.getGoal().in(Radians);
        Pose3d arm = new Pose3d(new Translation3d(0, 0, 0.25+height), new Rotation3d(0, angleRads, 0));

        Logger.recordOutput("Visualization/ArmAndEndEffector", arm);
    }
}
