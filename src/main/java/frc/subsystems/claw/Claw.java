package frc.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.Subsystems.ClawConstants;
import frc.util.Utils;
import frc.util.visualization.RobotVisualizer;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    private double offset;

    private ClawIO io;
    private ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
    private static Claw instance;

    private Claw() {
        io = (ClawIO) Utils.getIOImplementation(ClawIOReal.class, ClawIOSim.class,
            ClawIO.class);
    }

    public static Claw getInstance() {
        if (instance == null) instance = new Claw();
        return instance;
    }

    private void goToPosition(double position) {
        io.setPosition(position);
    }

    /**
     * @return Command that moves the claw to interfere with the note path. intended to be
     *         used to prevent note from contacting flywheels
     */
    public Command reverse() {
        return new InstantCommand(() -> {
            offset += 0.05;
            goToPosition(getPosition() - 0.05d);
        });
    }

    /**
     * @return Command that fires note by running the claw 1 full rotation.
     */
    public Command fire() {
        return new InstantCommand(() -> {
            double oldOffset = offset - (1 / 120.0d) * (offset / 0.05d);
            offset = 0;
            io.setEncoderPosition(0);
            run(() -> goToPosition(ClawConstants.MoveDistance + oldOffset))
                .andThen(new WaitCommand(0.5)).andThen(run(() -> goToPosition(0)))
                .schedule();
            RobotVisualizer.launchNote();
        });
    }

    /**
     * @return current position of the claw.
     */
    private double getPosition() { return inputs.position; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
    }
}
