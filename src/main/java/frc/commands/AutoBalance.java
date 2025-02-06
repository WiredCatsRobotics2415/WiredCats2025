package frc.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements.BalanceConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;

public class AutoBalance extends Command {
    CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    Pigeon2 pigeon = drive.getPigeon2();

    public void doBalance() {
        // i need to understand the climber fully to do this, which I do not yet -celeste
    }

    public StatusSignal<Angle> checkYaw() {
        if (pigeon.getYaw().getValueAsDouble() > BalanceConstants.yawThreshold) {
            return pigeon.getYaw();
            // i know this does nothing, we have to go rn
        } else {
            return pigeon.getYaw();
        }
    }

    @Override
    public void initialize() {

    }

}
