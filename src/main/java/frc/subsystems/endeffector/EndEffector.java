package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.vision.Vision;
import frc.utils.Util;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
    private static EndEffector instance;

    @Getter private boolean intakingCoral = false;
    @Getter private boolean intakingAlgae = false;
    @Getter private boolean outtaking = false;

    private EndEffector() {
        io = (EndEffectorIO) Util.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            EndEffectorIO.class);

        new Trigger(this::hasCoral).onTrue(turnOff());
        new Trigger(this::hasAlgae).onTrue(turnOff());
    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    public Command toggleIntakeCoral() {
        return runOnce(() -> {
            if (!intakingCoral) {
                io.setPower(EndEffectorConstants.IntakeCoralSpeed);
                intakingCoral = true;
            } else {
                io.setPower(0);
                intakingCoral = false;
            }
            intakingAlgae = false;
            outtaking = false;
        });
    }

    public Command toggleIntakeAlgae() {
        return runOnce(() -> {
            if (!intakingAlgae) {
                io.setPower(EndEffectorConstants.IntakeAlgaeSpeed);
                intakingAlgae = true;
            } else {
                io.setPower(0);
                intakingAlgae = false;
            }
            intakingCoral = false;
            outtaking = false;
        });
    }

    public Command outtake() {
        return runOnce(() -> {
            io.setPower(EndEffectorConstants.OuttakeSpeed);
            outtaking = true;
            intakingCoral = false;
            intakingAlgae = false;
        });
    }

    public Command turnOff() {
        return runOnce(() -> {
            io.setPower(0);
            outtaking = false;
            intakingCoral = false;
            intakingAlgae = false;
        });
    }

    private boolean sensorTrigger() {
        return inputs.sensorValue > EndEffectorConstants.IRThreshold;
    }

    private boolean cameraTrigger() {
        return Vision.getInstance()
            .getEndEffectorCameraAveragePixelValue() > EndEffectorConstants.AlgaeIntookCameraThreshold;
    }

    public boolean hasCoral() {
        return sensorTrigger() && intakingCoral;
    }

    public boolean hasAlgae() {
        return cameraTrigger() && intakingAlgae;
    }

    public Command intakeAndWaitForCoral() {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            intakingCoral = true;
        }), toggleIntakeCoral(), new WaitUntilCommand(() -> hasCoral()), new InstantCommand(() -> {
            intakingCoral = false;
        }), turnOff());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }
}
