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
    }

    public static CoralIntake getInstance() {
        if (instance == null) instance = new CoralIntake();
        return instance;
    }

    public Command turnOff() {
        return runOnce(() -> {
            isBeingIntook = false;
            state = false;
            io.setPower(0);
        });
    }

    public Command outtake() {
        return runOnce(() -> {
            isBeingIntook = false;
            io.setPower(CoralIntakeConstants.OuttakeSpeed);
        });
    }

    public Command intake() {
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
                turnOff().schedule();
                state = false;
            } else {
                intakeAndWaitForCoral().schedule();
                state = true;
            }
        });
    }

    public Command intakeAndWaitForCoral() {
        return new SequentialCommandGroup(intake(), new InstantCommand(() -> {
            state = true;
        }), new WaitUntilCommand(() -> hasCoral()), new InstantCommand(() -> {
            state = false;
        }), turnOff());
    }

    /**
     * uses limit switch to check if Coral is being intook
     */
    public boolean hasCoral() {
        return sensorTrigger() && isBeingIntook;
    }

    public boolean sensorTrigger() {
        return inputs.sensorValue > CoralIntakeConstants.IRThreshold;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
