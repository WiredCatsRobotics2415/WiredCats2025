package frc.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.constants.Subsystems.ElevatorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;

import org.littletonrobotics.junction.Logger;

/**
 * Reads from the elevator and publishes pose to NT for advantagescope
 */
public class Visualizer {

    private static Elevator elevatorSubsystem = Elevator.getInstance();
    private static Arm armSubsystem = Arm.getInstance();

    public static void update() {
        //Elevator
        double height = elevatorSubsystem.getHeightInches();
        Pose3d elevatorStage1 = new Pose3d(0, 0, 0, Rotation3d.kZero);
        Pose3d elevatorStage2 = new Pose3d(0, (height/ElevatorConstants.MaxHeight) + (ElevatorConstants.Stage1Height + ElevatorConstants.Stage2Height), 0, Rotation3d.kZero);
        Pose3d elevatorStage3 = new Pose3d(0, (height/ElevatorConstants.MaxHeight) + (ElevatorConstants.Stage1Height + ElevatorConstants.Stage2Height + ElevatorConstants.Stage3Height), 0, Rotation3d.kZero);
        Pose3d carriage = new Pose3d(0, height, 0, Rotation3d.kZero);

        Logger.recordOutput("Visualization/ElevatorStage1", elevatorStage1);
        Logger.recordOutput("Visualization/ElevatorStage2", elevatorStage2);
        Logger.recordOutput("Visualization/ElevatorStage3", elevatorStage3);
        Logger.recordOutput("Visualization/Carriage", carriage);

        //Arm
        double angle = armSubsystem.getAngleDegrees();
        Pose3d arm = new Pose3d(0, height, 0, new Rotation3d(0, Units.degreesToRadians(angle), 0));

        Logger.recordOutput("Visualization/Arm", arm);
    }
}
