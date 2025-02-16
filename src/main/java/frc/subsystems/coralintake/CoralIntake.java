package frc.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.endeffector.EndEffector;
import frc.utils.Util;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private static CoralIntake instance;

    @Getter private boolean intaking = false;

    private CoralIntake() {
        io = (CoralIntakeIO) Util.getIOImplementation(CoralIntakeIOReal.class, CoralIntakeIOSim.class,
            CoralIntakeIO.class);

        new Trigger(EndEffector.getInstance()::hasCoral).onTrue(turnOff());
    }

    public static CoralIntake getInstance() {
        if (instance == null) instance = new CoralIntake();
        return instance;
    }

    public Command toggleIntake() {
        return runOnce(() -> {
            if (!intaking) {
                io.setPower(CoralIntakeConstants.IntakeSpeed);
                intaking = true;
            } else {
                io.setPower(CoralIntakeConstants.OuttakeSpeed);
                intaking = false;
            }
        });
    }

    public Command turnOff() {
        return runOnce(() -> {
            io.setPower(0);
            intaking = false;
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);
    }
}
