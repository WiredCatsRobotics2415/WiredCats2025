package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.vision.Vision;
import frc.utils.TorqueMonitor;
import frc.utils.Util;
import frc.utils.tuning.TuneableNumber;
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

    private TuneableNumber proportionOfArmVeloToHoldCoralVolts = new TuneableNumber(-0.05,
        "EndEffector/proportionOfArmVeloToHoldCoralVolts");

    private TorqueMonitor coralIntakingTorqueMonitor;

    private EndEffector() {
        io = (EndEffectorIO) Util.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            new EndEffectorIO() {});

        coralIntakingTorqueMonitor = new TorqueMonitor(EndEffectorConstants.TorqueMonitorJumpThreshold,
            EndEffectorConstants.TorqueMonitorJumpMagnitude, EndEffectorConstants.TorqueMonitorTripTime);
        // new Trigger(this::hasCoral).onTrue(Commands.runOnce(() -> {
        // System.out.println("going to hold coral");
        // io.setVoltage(EndEffectorConstants.HoldCoralVolts.get());
        // intakingCoral = false;
        // intakingAlgae = false;
        // outtakingCoral = false;
        // outtakingAlgae = false;
        // }));
        new Trigger(this::hasAlgae).onTrue(Commands.runOnce(() -> {
            io.setVoltage(EndEffectorConstants.HoldAlgaeVolts.get());
            intakingCoral = false;
            intakingAlgae = false;
            outtakingCoral = false;
            outtakingAlgae = false;
        }));
    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    public Command toggleIntakeCoral() {
        return runOnce(() -> {
            if (intakingCoral) {
                io.setVoltage(0);
                intakingCoral = false;
                System.out.println("EE: set intake coral to false!");
            } else {
                io.setVoltage(EndEffectorConstants.IntakeCoralVolts.get());
                intakingCoral = true;
                System.out.println("EE: set intake coral to true!");
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

    public Command intakeAlgae() {
        return runOnce(() -> {
            io.setVoltage(EndEffectorConstants.IntakeAlgaeVolts.get());
            intakingCoral = false;
            intakingAlgae = true;
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
                System.out.println("intaking algae");
                io.setVoltage(EndEffectorConstants.IntakeAlgaeVolts.get());
                intakingAlgae = true;
            } else {
                System.out.println("no longer intaking algae");
                io.setVoltage(0);
                intakingAlgae = false;
            }
            intakingCoral = false;
            outtakingAlgae = false;
            outtakingCoral = false;
        });
    }

    public Command toggleOuttakeAlgae() {
        return runOnce(() -> {
            coralIntakingTorqueMonitor.reset();
            if (outtakingAlgae) {
                io.setVoltage(0);
                intakingCoral = false;
                intakingAlgae = false;
                outtakingCoral = false;
                outtakingAlgae = false;
                return;
            }
            io.setVoltage(EndEffectorConstants.OuttakeAlageVolts.get());
            intakingCoral = false;
            intakingAlgae = false;
            outtakingCoral = false;
            outtakingAlgae = true;
        });
    }

    public Command toggleOuttakeCoral() {
        return runOnce(() -> {
            coralIntakingTorqueMonitor.reset();
            if (outtakingCoral) {
                io.setVoltage(0);
                intakingCoral = false;
                intakingAlgae = false;
                outtakingAlgae = false;
                outtakingCoral = false;
                return;
            }
            io.setVoltage(EndEffectorConstants.OuttakeCoralVolts.get());
            intakingCoral = false;
            intakingAlgae = false;
            outtakingAlgae = false;
            outtakingCoral = true;
        });
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
            .getEndEffectorCameraAveragePixelValue() < EndEffectorConstants.AlgaeIntookCameraThreshold.get();
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
