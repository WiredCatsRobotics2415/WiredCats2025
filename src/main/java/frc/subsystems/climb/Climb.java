package frc.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.utils.Util;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private ClimbIO io;

    private Climb() {
        io = (ClimbIO) Util.getIOImplementation(ClimbIOReal.class, ClimbIOSim.class, new ClimbIO() {});
    }

    public static Climb getInstance() {
        if (instance == null) instance = new Climb();
        return instance;
    }

    public Command runForward() {
        return new InstantCommand(() -> io.setVoltage(ClimberConstants.ForwardVolts.get()));
    }

    public Command runBackward() {
        return new InstantCommand(() -> io.setVoltage(ClimberConstants.BackwardVolts.get()));
    }

    public Command stop() {
        return new InstantCommand(() -> io.setVoltage(0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
