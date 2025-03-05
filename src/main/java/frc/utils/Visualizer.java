package frc.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.constants.Measurements;
import frc.constants.Subsystems.ElevatorConstants;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.vision.Vision;
import frc.utils.math.Trig;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Reads from the elevator and publishes pose to NT for advantagescope
 */
public class Visualizer {
    private static Elevator elevatorSubsystem = Elevator.getInstance();
    private static Arm armSubsystem = Arm.getInstance();
    private static CoralIntake coralIntakeSubsystem = CoralIntake.getInstance();
    private static Vision vision = Vision.getInstance();

    private static Color8Bit goalColor = new Color8Bit(Color.kOrange);
    private static Color8Bit actualColor = new Color8Bit(Color.kBlack);

    private static LoggedMechanism2d coralIntakeMech2d = new LoggedMechanism2d(0, 3);
    private static LoggedMechanismRoot2d coralIntakeRoot = coralIntakeMech2d.getRoot("CoralIntake",
        Units.inchesToMeters(-10.394), Units.inchesToMeters(0.5));
    private static LoggedMechanismLigament2d coralIntake = coralIntakeRoot
        .append(new LoggedMechanismLigament2d("Pivot", Units.inchesToMeters(21), 0, 3, goalColor));

    private static Pose3d visualizeElevator(Distance height, String key) {
        Pose3d elevatorBase = Pose3d.kZero;

        double offsetOfStage2 = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight).times(ElevatorConstants.Stage2Height).in(Meters);
        Pose3d elevatorStage2 = new Pose3d(offsetOfStage2, 0,
            height.div(ElevatorConstants.MaxHeight).times(ElevatorConstants.Stage2Height).in(Meters), Rotation3d.kZero);

        double offsetOfStage3 = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight).times(ElevatorConstants.Stage3Height).in(Meters);
        Pose3d elevatorStage3 = new Pose3d(offsetOfStage2 + offsetOfStage3, 0,
            height.div(ElevatorConstants.MaxHeight)
                .times(ElevatorConstants.Stage2Height.plus(ElevatorConstants.Stage3Height)).in(Meters),
            Rotation3d.kZero);

        double carriageOffset = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight).times(ElevatorConstants.Stage2Height).in(Meters);
        Pose3d carriage = new Pose3d(offsetOfStage2 + offsetOfStage3 + carriageOffset, 0, height.in(Meters),
            Rotation3d.kZero);

        Logger.recordOutput("Visualization/" + key + "/ElevatorBase", elevatorBase);
        Logger.recordOutput("Visualization/" + key + "/ElevatorStage2", elevatorStage2);
        Logger.recordOutput("Visualization/" + key + "/ElevatorStage3", elevatorStage3);
        Logger.recordOutput("Visualization/" + key + "/Carriage", carriage);
        return carriage;
    }

    private static void visualizeArm(Angle angle, Pose3d carriage, String key) {
        Pose3d arm = carriage
            .plus(new Transform3d(new Translation3d(-0.044, 0, 0.25), new Rotation3d(0, -angle.in(Radians), 0)));

        Logger.recordOutput("Visualization/" + key + "/ArmAndEndEffector", arm);
    }

    private static void visualizeCoralIntake(Angle angle, String key, Color8Bit color) {
        coralIntake.setAngle(angle.in(Degrees));
        coralIntake.setColor(color);
        Logger.recordOutput("Visualization/" + key + "/CoralIntake", coralIntakeMech2d);
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
        Pose3d carraigeGoal = visualizeElevator(elevatorGoal, "Goal");
        Pose3d carraigeActual = visualizeElevator(elevatorActual, "Actual");

        Angle armGoal = armSubsystem.getGoal();
        Angle armActual = armSubsystem.getMeasurement();
        visualizeArm(armGoal, carraigeGoal, "Goal");
        visualizeArm(armActual, carraigeActual, "Actual");

        Angle coralIntakeGoal = coralIntakeSubsystem.getGoal();
        Angle coralIntakeActual = coralIntakeSubsystem.getPivotAngle();
        visualizeCoralIntake(coralIntakeGoal, "Goal", goalColor);
        visualizeCoralIntake(coralIntakeActual, "Actual", actualColor);

        Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);

        visualizePECameraTargets();
    }
}
