package frc.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.constants.Measurements;
import frc.constants.Subsystems.ElevatorConstants;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.vision.Vision;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * Reads from the elevator and publishes pose to NT for advantagescope
 */
public class Visualizer {
    private static Elevator elevatorSubsystem = Elevator.getInstance();
    private static Arm armSubsystem = Arm.getInstance();
    private static Vision vision = Vision.getInstance();

    private static void visualizeElevator(Distance height, String key) {
        Pose3d elevatorBase = new Pose3d(0, 0, 0, Rotation3d.kZero);
        Pose3d elevatorStage2 = new Pose3d(0, 0,
            height.div(ElevatorConstants.MaxHeight).times(ElevatorConstants.Stage2Height).in(Meters), Rotation3d.kZero);
        Pose3d elevatorStage3 = new Pose3d(0, 0,
            height.div(ElevatorConstants.MaxHeight)
                .times(ElevatorConstants.Stage2Height.plus(ElevatorConstants.Stage3Height)).in(Meters),
            Rotation3d.kZero);
        Pose3d carriage = new Pose3d(0, 0, height.in(Meters), Rotation3d.kZero);

        Logger.recordOutput("Visualization/" + key + "/ElevatorBase", elevatorBase);
        Logger.recordOutput("Visualization/" + key + "/ElevatorStage2", elevatorStage2);
        Logger.recordOutput("Visualization/" + key + "/ElevatorStage3", elevatorStage3);
        Logger.recordOutput("Visualization/" + key + "/Carriage", carriage);
    }

    private static void visualizeArm(Angle angle, Distance elevatorHeight, String key) {
        Pose3d arm = new Pose3d(new Translation3d(0, 0, 0.25 + elevatorHeight.in(Meters)),
            new Rotation3d(0, angle.in(Radians), 0));

        Logger.recordOutput("Visualization/" + key + "/ArmAndEndEffector", arm);
    }

    private static void visualizePECameraTargets() {
        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            int tagId = vision.getClosestApriltagTo(i);
            if (tagId == -1) {
                Logger.recordOutput("Visualization/" + VisionConstants.PoseEstimationLLNames[i] + "/NearestTag",
                    Pose3d.kZero);
            } else {
                Logger.recordOutput("Visualization/" + VisionConstants.PoseEstimationLLNames[i] + "/NearestTag",
                    Measurements.ApriltagFieldLayout.getTagPose(tagId).orElse(Pose3d.kZero));
            }
        }
    }

    public static void update() {
        Distance elevatorGoal = elevatorSubsystem.getGoal();
        Distance elevatorActual = elevatorSubsystem.getMeasurement();
        visualizeElevator(elevatorGoal, "Goal");
        visualizeElevator(elevatorActual, "Actual");

        Angle armGoal = armSubsystem.getGoal();
        Angle armActual = armSubsystem.getMeasurement();
        visualizeArm(armGoal, elevatorGoal, "Goal");
        visualizeArm(armActual, elevatorActual, "Actual");

        Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);

        visualizePECameraTargets();
    }
}
