package frc.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.Util;
import frc.utils.driver.DashboardManager;
import frc.utils.driver.DashboardManager.LayoutConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
    @Getter private boolean isBeingIntook = false;

    @Getter private boolean state = false;

    private CoralIntakeIO io;
    private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private static CoralIntake instance;

    private CoralIntake() {
        io = (CoralIntakeIO) Util.getIOImplementation(CoralIntakeIOReal.class, CoralIntakeIOSim.class,
            CoralIntakeIO.class);

        DashboardManager.getInstance().addBoolSupplier(true, "Intaking", () -> state, LayoutConstants.IntakeStatus);
        DashboardManager.getInstance().addBoolSupplier(true, "Intook", () -> sensorTrigger(),
            LayoutConstants.CoralIntook);
    }

    public static CoralIntake getInstance() {
        if (instance == null) instance = new CoralIntake();
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
            io.setPower(CoralIntakeConstants.OuttakeSpeed);
        });
    }

    public Command in() {
        return runOnce(() -> {
            isBeingIntook = true;
            io.setPower(CoralIntakeConstants.IntakeSpeed);
        });
    }

    /**
     * Command that toggles between intaking and not intaking
     */
    public Command toggleCoralIntake() {
        return runOnce(() -> {
            if (state == true) {
                off().schedule();
                state = false;
            } else {
                IntakeAndWaitForCoral().schedule();
                state = true;
            }
        });
    }

    public Command IntakeAndWaitForCoral() {
        return new SequentialCommandGroup(in(), new InstantCommand(() -> {
            state = true;
        }), new WaitUntilCommand(() -> hasCoral()), new InstantCommand(() -> {
            state = false;
        }), off());
    }

    /**
     * uses limit switch to check if Coral is being intook
     */
    public boolean hasCoral() {
        return inputs.limitSwitch && isBeingIntook;
    }

    /**
     * return true if the limitswitch is triggered
     */
    public boolean sensorTrigger() {
        return inputs.limitSwitch;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
