package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
            new EndEffectorIO() {});

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

    public Command intakeAndWaitForCoral() {
        return run(() -> {
            io.setPower(EndEffectorConstants.IntakeCoralSpeed);
            intakingCoral = true;
            intakingAlgae = false;
            outtaking = false;
        }).until(this::hasCoral).andThen(turnOff());
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

    /**
     * Outtakes, spinning at different speeds/directions depending on which sensor is triggered. Will not run if neither sensor is triggered.
     */
    public Command toggleOuttake() {
        return runOnce(() -> {
            if (outtaking) {
                io.setPower(0);
                outtaking = false;
                return;
            }
            if (sensorTrigger()) {
                io.setPower(EndEffectorConstants.OuttakeCoralSpeed);
                intakingCoral = false;
                intakingAlgae = false;
                outtaking = true;
            } else if (cameraTrigger()) {
                io.setPower(EndEffectorConstants.OuttakeAlageSpeed);
                intakingCoral = false;
                intakingAlgae = false;
                outtaking = true;
            }
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }
}
