package frc.subsystems.algaeintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.Subsystems.AlgaeIntakeConstants;
import frc.utils.Utils;
import frc.utils.driver.DashboardManager;
import frc.utils.driver.DashboardManager.LayoutConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
    @Getter private boolean isBeingIntook = false;

    @Getter private boolean state = false;

    private AlgaeIntakeIO io;
    private AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    private static AlgaeIntake instance;

    private AlgaeIntake() {
        io = (AlgaeIntakeIO) Utils.getIOImplementation(AlgaeIntakeIOReal.class, AlgaeIntakeIOSim.class,
            AlgaeIntakeIO.class);

        DashboardManager.getInstance().addBoolSupplier(true, "Intaking", () -> state, LayoutConstants.IntakeStatus);
        DashboardManager.getInstance().addBoolSupplier(true, "Intook", () -> sensorTrigger(), LayoutConstants.Intook);
    }

    public static AlgaeIntake getInstance() {
        if (instance == null) instance = new AlgaeIntake();
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
            io.setPower(AlgaeIntakeConstants.OuttakeSpeed);
        });
    }

    public Command in() {
        return runOnce(() -> {
            isBeingIntook = true;
            io.setPower(AlgaeIntakeConstants.IntakeSpeed);
        });
    }

    /**
     * Command that toggles between intaking and not intaking
     */
    public Command toggleAlgaeIntake() {
        return runOnce(() -> {
            if (state == true) {
                off().schedule();
                state = false;
            } else {
                IntakeAndWaitForAlgae().schedule();
                state = true;
            }
        });
    }

    public Command IntakeAndWaitForAlgae() {
        return new SequentialCommandGroup(in(), new InstantCommand(() -> {
            state = true;
        }), new WaitUntilCommand(() -> hasAlgae()), new InstantCommand(() -> {
            state = false;
        }), off());
    }

    /**
     * uses limit switch to check if algae is being intook
     */
    public boolean hasAlgae() {
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
