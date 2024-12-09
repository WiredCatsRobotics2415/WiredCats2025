package frc.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.constants.Measurements;
import frc.constants.TunerConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.intake.Intake;
import frc.util.visualization.NoteVisualizer;
import frc.util.visualization.NoteVisualizer.Note;
import frc.util.visualization.RobotVisualizer;
import lombok.Getter;

public class SimulatedEnvironment {
    @Getter private double[] currentNoteTargetInfo;

    private final double maxNoteVisibleDistance = Units.feetToMeters(4.5);
    private final double maxNoteVisibleAngle = Units.degreesToRadians(20);

    public SimulatedEnvironment() {
        NoteVisualizer.renderNotes.schedule();
    }

    public void updateEnvironment() {
        Pose2d robotPosition = TunerConstants.DriveTrain.getState().Pose;
        Translation2d intakePosition = robotPosition.plus(Measurements.IntakeDownTransform)
            .getTranslation();

        boolean canIntake = !RobotVisualizer.hasNote()
            && Arm.getInstance().getGoalInDegrees() < 1.0d
            && (Intake.getInstance().isBeingIntook() || Intake.getInstance().isState());

        boolean didSeeNote = false;
        for (Note note : NoteVisualizer.getCurrentNotes()) {
            Translation2d noteTranslation = note.getPose().getTranslation()
                .toTranslation2d();
            double distance = intakePosition.getDistance(noteTranslation);
            if (canIntake && distance <= Measurements.NoteDiameter * 1.5
                && !note.isBeingShot()) {
                RobotVisualizer.takeNote(note);
            }
            if (distance < maxNoteVisibleDistance) {
                double angleRad = Math.atan2(noteTranslation.getY() - robotPosition.getY(),
                    noteTranslation.getX() - robotPosition.getX());
                double diff = robotPosition.getRotation().getRadians() - angleRad;
                if (Math.abs(diff) < maxNoteVisibleAngle) {
                    currentNoteTargetInfo = new double[] { 1.0, diff };
                    didSeeNote = true;
                }
            }
        }
        if (!didSeeNote) currentNoteTargetInfo = new double[] { 0.0, 0.0 };
    }

    public void resetField(boolean includePreload) {
        NoteVisualizer.clearNotes();
        RobotVisualizer.removeNote();
        for (Translation3d notePosition : Measurements.DefaultFieldNotes) {
            new Note(notePosition); // Just constructing it automatically adds it to
            // currentNotes
        }
        if (includePreload) RobotVisualizer.takeNote(new Note(new Translation3d(0, 0, 0)));
    }
}
