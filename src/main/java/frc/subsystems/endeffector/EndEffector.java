package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.vision.Vision;
import frc.utils.TorqueMonitor;
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

    private TorqueMonitor coralIntakingTorqueMonitor;

    private EndEffector() {
        io = (EndEffectorIO) Util.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            new EndEffectorIO() {});

        coralIntakingTorqueMonitor = new TorqueMonitor(EndEffectorConstants.TorqueMonitorJumpThreshold,
            EndEffectorConstants.TorqueMonitorJumpMagnitude, EndEffectorConstants.TorqueMonitorTripTime);
        new Trigger(this::hasCoral).onTrue(Commands.runOnce(() -> {
            io.setVoltage(EndEffectorConstants.HoldCoralVolts.get());
            intakingCoral = false;
        }));
        new Trigger(this::hasAlgae).onTrue(Commands.runOnce(() -> {
            io.setVoltage(EndEffectorConstants.HoldAlgaeVolts.get());
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
                io.setVoltage(EndEffectorConstants.IntakeCoralVolts.get());
                intakingCoral = true;
                System.out.println("EE: set intake coral to true!");
            } else {
                io.setVoltage(0);
                intakingCoral = false;
                System.out.println("EE: set intake coral to false!");
            }
            intakingAlgae = false;
            outtakingCoral = false;
            outtakingAlgae = false;
        });
    }

    public Command intakeCoral() {
        return runOnce(() -> {
            io.setVoltage(EndEffectorConstants.IntakeCoralVolts.get());
            intakingCoral = true;
            intakingAlgae = false;
            outtakingAlgae = false;
            outtakingCoral = false;
        });
    }

    public Command intakeAndWaitForCoral() {
        return run(() -> {
            io.setVoltage(EndEffectorConstants.IntakeCoralVolts.get());
            intakingCoral = true;
            intakingAlgae = false;
            outtakingAlgae = false;
            outtakingCoral = false;
        }).until(this::hasCoral).andThen(turnOff());
    }

    public Command toggleIntakeAlgae() {
        return runOnce(() -> {
            if (!intakingAlgae) {
                io.setVoltage(EndEffectorConstants.IntakeAlgaeVolts.get());
                intakingAlgae = true;
            } else {
                io.setVoltage(0);
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
            coralIntakingTorqueMonitor.reset();
            if (outtakingAlgae || outtakingCoral) {
                io.setVoltage(0);
                outtakingAlgae = false;
                outtakingCoral = false;
                return;
            }
            if (algaeSensorTrigger()) {
                io.setVoltage(EndEffectorConstants.OuttakeAlageVolts.get());
                intakingCoral = false;
                intakingAlgae = false;
                outtakingAlgae = true;
                return;
            }
            io.setVoltage(EndEffectorConstants.OuttakeCoralVolts.get());
            intakingCoral = false;
            intakingAlgae = false;
            outtakingCoral = true;
        });
    }

    public Command shootCoral() {
        return toggleOuttake().andThen(Commands.waitSeconds(1.25)).andThen(toggleOuttake());
    }

    public Command turnOff() {
        return runOnce(() -> {
            io.setVoltage(0);
            outtakingAlgae = false;
            outtakingCoral = false;
            intakingCoral = false;
            intakingAlgae = false;
        });
    }

    public boolean coralSensorTrigger() {
        return coralIntakingTorqueMonitor.isTripped();
        // TODO: uncomment when ir sensor is replaced
        // return inputs.sensorValue >= EndEffectorConstants.IRThreshold.get();
    }

    public boolean algaeSensorTrigger() {
        return Vision.getInstance()
            .getEndEffectorCameraAveragePixelValue() > EndEffectorConstants.AlgaeIntookCameraThreshold.get();
    }

    public boolean hasCoral() {
        return coralSensorTrigger() && intakingCoral;
    }

    public boolean hasAlgae() {
        return algaeSensorTrigger() && intakingAlgae;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);

        coralIntakingTorqueMonitor.update(inputs.motorStatorCurrent);

        Logger.recordOutput("EndEffector/containingCoral", coralSensorTrigger());
        Logger.recordOutput("EndEffector/containingAlgae", algaeSensorTrigger());
    }
}
