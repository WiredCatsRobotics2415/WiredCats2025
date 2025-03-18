package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    @Getter private boolean outtakingCoral = false;
    @Getter private boolean outtakingAlgae = false;

    private EndEffector() {
        io = (EndEffectorIO) Util.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            new EndEffectorIO() {});
        
        new Trigger(this::hasCoral).onTrue(Commands.runOnce(() -> {
            io.setPower(EndEffectorConstants.HoldCoralSpeed.get());
            intakingCoral = false;
        }));
        new Trigger(this::hasAlgae).onTrue(Commands.runOnce(() -> {
            io.setPower(EndEffectorConstants.HoldAlgaeSpeed.get());
            intakingAlgae = false;
        }));
    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    public Command toggleIntakeCoral() {
        return runOnce(() -> {
            if (!intakingCoral) {
                io.setPower(EndEffectorConstants.IntakeCoralSpeed.get());
                intakingCoral = true;
            } else {
                io.setPower(0);
                intakingCoral = false;
            }
            intakingAlgae = false;
            outtakingCoral = false;
            outtakingAlgae = false;
        });
    }

    public Command intakeAndWaitForCoral() {
        return run(() -> {
            io.setPower(EndEffectorConstants.IntakeCoralSpeed.get());
            intakingCoral = true;
            intakingAlgae = false;
            outtakingAlgae = false;
            outtakingCoral = false;
        }).until(this::hasCoral).andThen(turnOff());
    }

    public Command toggleIntakeAlgae() {
        return runOnce(() -> {
            if (!intakingAlgae) {
                io.setPower(EndEffectorConstants.IntakeAlgaeSpeed.get());
                intakingAlgae = true;
            } else {
                io.setPower(0);
                intakingAlgae = false;
            }
            intakingCoral = false;
            outtakingAlgae = false;
            outtakingCoral = false;
        });
    }

    /**
     * Outtakes, spinning at different speeds/directions depending on which sensor is triggered. Will not run if neither sensor is triggered.
     */
    public Command toggleOuttake() {
        return runOnce(() -> {
            if (outtakingAlgae || outtakingCoral) {
                io.setPower(0);
                outtakingAlgae = false;
                outtakingCoral = false;
                return;
            }
            if (cameraTrigger()) {
                io.setPower(EndEffectorConstants.OuttakeAlageSpeed.get());
                intakingCoral = false;
                intakingAlgae = false;
                outtakingAlgae = true;
                return;
            }
            io.setPower(EndEffectorConstants.OuttakeCoralSpeed.get());
            intakingCoral = false;
            intakingAlgae = false;
            outtakingCoral = true;
        });
    }

    public Command turnOff() {
        return runOnce(() -> {
            io.setPower(0);
            outtakingAlgae = false;
            outtakingCoral = false;
            intakingCoral = false;
            intakingAlgae = false;
        });
    }

    public boolean irSensorTrigger() {
        return inputs.sensorValue >= EndEffectorConstants.IRThreshold.get();
    }

    public boolean cameraTrigger() {
        return Vision.getInstance()
            .getEndEffectorCameraAveragePixelValue() > EndEffectorConstants.AlgaeIntookCameraThreshold.get();
    }

    public boolean hasCoral() {
        return irSensorTrigger() && intakingCoral;
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
