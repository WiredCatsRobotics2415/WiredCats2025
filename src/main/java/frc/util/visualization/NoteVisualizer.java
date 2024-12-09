package frc.util.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.util.math.ProjectileMotion3d;
import java.util.ArrayList;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
    private static final double shotSpeed = 8.5; // Meters per sec
    private static final double timeScale = 0.2; // Time to multiply timer.get calls by;
                                                 // speeds up or slows down a shot but does
                                                 // not impact velocity
    // private static final double shotSpeed = 0.5; // Meters per sec
    private static final double trajectoryTimestep = 0.05;

    @Getter private static ArrayList<Note> currentNotes = new ArrayList<Note>();

    public static Command shoot(Note note) {
        return new ScheduleCommand(Commands.defer(() -> {
            note.setBeingShot(true);
            Pose3d notePose = RobotVisualizer.getCurrentNotePose3d();
            ProjectileMotion3d projectileMotion3d = new ProjectileMotion3d(notePose,
                shotSpeed);

            Timer timer = new Timer();
            timer.start();
            return Commands.run(() -> {
                note.setPose(projectileMotion3d.getPoseAtTime(timer.get() * timeScale));
            }).until(() -> note.getPose().getZ() < 0).finallyDo(() -> {
                currentNotes.remove(note);
            });
        }, Set.of()).ignoringDisable(true));
    }

    public static void updateNoteTrajectory(Note note) {
        ProjectileMotion3d projectileMotion3d = new ProjectileMotion3d(
            RobotVisualizer.getCurrentNotePose3d(), shotSpeed);
        ArrayList<Pose3d> posesList = new ArrayList<Pose3d>();
        for (double i = 0; i < 5; i += trajectoryTimestep) {
            Pose3d toAdd = projectileMotion3d.getPoseAtTime(i);
            posesList.add(toAdd);
            if (toAdd.getZ() <= 0) break;
        }
        Pose3d[] poses = new Pose3d[posesList.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = posesList.get(i);
        Logger.recordOutput("NoteShot", poses);
    }

    public static Command renderNotes = new Command() {
        public void execute() {
            Pose3d[] poses = new Pose3d[currentNotes.size()];
            for (int i = 0; i < poses.length; i++)
                poses[i] = currentNotes.get(i).getPose();
            Logger.recordOutput("NoteVisualizer", poses);
        }

        public boolean isFinished() { return false; };
    };

    static {
        renderNotes = renderNotes.ignoringDisable(true);
    }

    public static void clearNotes() {
        currentNotes.clear();
    }

    public static class Note {
        @Getter
        @Setter private Pose3d pose;

        @Getter
        @Setter private boolean isBeingShot = false;

        public Note(Translation3d position) {
            pose = new Pose3d(position, new Rotation3d());
            currentNotes.add(this);
        }
    }
}
