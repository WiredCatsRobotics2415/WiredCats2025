package frc.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.Subsystems.IntakeConstants;
import frc.util.Utils;
import frc.util.driver.DashboardManager;
import frc.util.driver.DashboardManager.LayoutConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    @Getter private boolean isBeingIntook = false;

    @Getter private boolean state = false;

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private static Intake instance;

    private Intake() {
        io = (IntakeIO) Utils.getIOImplementation(IntakeIOReal.class, IntakeIOSim.class,
            IntakeIO.class);

        DashboardManager.getInstance().addBoolSupplier(true, "Intaking", () -> state,
            LayoutConstants.IntakeStatus);
        DashboardManager.getInstance().addBoolSupplier(true, "Intook",
            () -> sensorTrigger(), LayoutConstants.Intook);
    }

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    public Command off() {
        return runOnce(() -> {
            isBeingIntook = false;
            state = false;
            io.off();
        });
    }

    public Command out() {
        return runOnce(() -> {
            isBeingIntook = false;
            io.on(IntakeConstants.OuttakeSpeed);
        });
    }

    public Command in() {
        return runOnce(() -> {
            isBeingIntook = true;
            io.on(IntakeConstants.IntakeSpeed);
        });
    }

    public Command stopNoteForShooting() {
        return runOnce(() -> {
            isBeingIntook = false;
            io.off();
        });
    }

    /**
     * @return Command that toggles between intaking and not intaking. Does NOT take into
     *         account other motor modes, ie. If this has been called, and the out Command
     *         is run, then when this command is run again, the motor will just be turned
     *         off.
     */
    public Command toggleIntake() {
        return runOnce(() -> {
            if (state == true) {
                off().schedule();
                state = false;
            } else {
                intakeAndWaitForNote().schedule();
                state = true;
            }
        });
    }

    public Command intakeAndWaitForNote() {
        return new SequentialCommandGroup(in(), new InstantCommand(() -> {
            state = true;
        }), new WaitUntilCommand(() -> hasNote()), new InstantCommand(() -> {
            state = false;
        }),
            // new WaitCommand(0.2),
            stopNoteForShooting());
    }

    /**
     * @return true if the note has been intook (used to signal intake off)
     */
    public boolean hasNote() {
        return inputs.sensorTrigger && isBeingIntook;
    }

    /**
     * @return true if the sensor is triggered (does not depend on intaking state)
     */
    public boolean sensorTrigger() {
        return inputs.sensorTrigger;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
