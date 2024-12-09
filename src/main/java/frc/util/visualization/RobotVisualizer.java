package frc.util.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.constants.Measurements;
import frc.constants.RuntimeConstants;
import frc.constants.TunerConstants;
import frc.robot.Robot;
import frc.util.visualization.NoteVisualizer.Note;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
    private static Translation3d armTranslation3d = new Translation3d(-0.25, 0, 0.245);
    private static Transform3d intookNoteOffset = new Transform3d(
        new Translation3d(0.625, 0, 0.02),
        new Rotation3d(0, Units.degreesToRadians(235), 0));

    @Getter private static Transform3d currentArmPose3d = new Transform3d();

    @Getter private static Pose3d currentNotePose3d = new Pose3d();

    @Getter private static Note currentNote = null;

    public static void update(double angle, double goal) {
        if (Robot.isReal() && !RuntimeConstants.visualizationEnabledWhenReal) return;
        // Arm Real
        Pose3d poseAngle = new Pose3d(armTranslation3d,
            new Rotation3d(0, -Units.degreesToRadians(angle), 0));
        currentArmPose3d = poseAngle.minus(new Pose3d());
        Logger.recordOutput("Arm/3dPoseAngle", poseAngle);

        // Arm Goal
        Pose3d poseGoal = new Pose3d(armTranslation3d,
            new Rotation3d(0, -Units.degreesToRadians(goal), 0));
        Logger.recordOutput("Arm/3dPoseGoal", poseGoal);

        Pose3d robotPosition = new Pose3d(TunerConstants.DriveTrain.getState().Pose);
        // Intook note
        if (currentNote != null) {
            Pose3d armPosition = robotPosition
                .plus(new Transform3d(poseAngle.getTranslation(), poseAngle.getRotation()));
            currentNotePose3d = armPosition.plus(intookNoteOffset);
            currentNote.setPose(currentNotePose3d);
            NoteVisualizer.updateNoteTrajectory(currentNote);
        }

        Logger.recordOutput("Camera/BackLL",
            robotPosition.plus(Measurements.BackLLTransform));
    }

    public static boolean hasNote() {
        return currentNote != null;
    }

    public static void takeNote(Note note) {
        currentNote = note;
    }

    public static void launchNote() {
        if (currentNote == null) return;
        NoteVisualizer.shoot(currentNote).schedule();
        currentNote = null;
    }

    public static void removeNote() {
        if (currentNote == null) return;
        currentNote = null;
    }
}
