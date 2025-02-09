package frc.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements.BalanceConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;

public class AutoBalance extends Command {
    CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    Pigeon2 pigeon = drive.getPigeon2();

    public void balanceRoll() {
        // requires testing of how to balance subsystems
    }

    public void balancePitch() {
        // requires testing of how to balance subsystems
    }

    public boolean checkRoll() {
        if (Math.abs(pigeon.getRoll().getValueAsDouble()) > BalanceConstants.rollThreshold) {
            return false;
        } else {
            return true;
        }
    }

    public boolean checkPitch() {
        if (Math.abs(pigeon.getPitch().getValueAsDouble()) > BalanceConstants.pitchThreshold) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!checkRoll() | !checkPitch()) {
            if (!checkRoll()) {
                // this should change our roll by very small increments depending on which direction it is tilted
                // how this works will be tested by driver first
                balanceRoll();
            }
            if (!checkPitch()) {
                // this should change our pitch by very small increments depending on which direction it is tilted
                // how this works will be tested by driver first
                balancePitch();
            }
        } else {
            end(false);
        }
    }

}
