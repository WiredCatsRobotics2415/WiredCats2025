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
import frc.robot.Robot;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.leds.LEDStrip;
import frc.subsystems.vision.Vision;
import frc.utils.math.Trig;
import frc.utils.tuning.*;
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
    private static LEDStrip ledStripSubsystem = LEDStrip.getInstance();

    private static LoggedMechanism2d ledStrip = new LoggedMechanism2d(1, 1);
    private static LoggedMechanismRoot2d ledStripRoot = ledStrip.getRoot("LEDStrip", 0.5, 0.2);
    private static LoggedMechanismLigament2d ledStripLine = ledStripRoot
        .append(new LoggedMechanismLigament2d("LEDStripLine", .35, 90, 6, new Color8Bit(Color.kBlack)));

    private static Pose3d visualizeElevator(Distance height, String key) {
        Pose3d elevatorBase = Pose3d.kZero;

        double offsetOfStage2 = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight.distance()).times(ElevatorConstants.Stage2Height).in(Meters);
        Pose3d elevatorStage2 = new Pose3d(offsetOfStage2, 0,
            height.div(ElevatorConstants.MaxHeight.distance()).times(ElevatorConstants.Stage2Height).in(Meters),
            Rotation3d.kZero);

        double offsetOfStage3 = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight.distance()).times(ElevatorConstants.Stage3Height).in(Meters);
        Pose3d elevatorStage3 = new Pose3d(offsetOfStage2 + offsetOfStage3, 0,
            height.div(ElevatorConstants.MaxHeight.distance())
                .times(ElevatorConstants.Stage2Height.plus(ElevatorConstants.Stage3Height)).in(Meters),
            Rotation3d.kZero);

        double carriageOffset = Trig.cosizzle(Units.degreesToRadians(90 - 3.7)) *
            height.div(ElevatorConstants.MaxHeight.distance()).times(ElevatorConstants.Stage2Height).in(Meters);
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

    private static void visualizeCoralIntake(Angle angle, String key) {
        Pose3d coralIntake = new Pose3d(new Translation3d(Inches.of(-8.75), Inches.of(3.25), Inches.of(6.5)),
            new Rotation3d(0, -angle.in(Radians), Math.PI));
        Logger.recordOutput("Visualization/" + key + "/CoralIntake", coralIntake);
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
        visualizeCoralIntake(coralIntakeGoal, "Goal");
        visualizeCoralIntake(coralIntakeActual, "Actual");

        if (Robot.isSimulation()) {
            Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
            Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);
        }

        visualizePECameraTargets();

        ledStripLine.setColor(new Color8Bit(ledStripSubsystem.getCurrentColor()));
        Logger.recordOutput("Visualization/LEDStrip", ledStrip);
    }

    public static void setLEDStripColor(Color8Bit color) {
        ledStripLine.setColor(color);
    }
}
