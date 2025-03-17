package frc.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableElevatorFF;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuneablePIDController;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private boolean coasting;
    private boolean hasResetPidController = false;
    @Getter private double goalInches;

    private double lastMeasurement = 0.0;
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();

    private TuneableElevatorFF ff = new TuneableElevatorFF(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA, "ElevatorFF");
    private TuneablePIDController pid = new TuneablePIDController(ElevatorConstants.kP, 0.0d, ElevatorConstants.kD,
        "ElevatorPID");
    @Getter private TuneablePIDController lowPid = new TuneablePIDController(ElevatorConstants.lowkP, 0.0d,
        ElevatorConstants.lowkD, "ElevatorLowPID");

    private TuneableNumber constant = new TuneableNumber(0.01, "Elevator/constant");
    private TuneableNumber stopMap = new TuneableNumber(40, "Elevator/stopMap");
    private TuneableNumber kg = new TuneableNumber(0.6, "Elevator/kg");

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, new ElevatorIO() {});
        pid.setTolerance(ElevatorConstants.BaseGoalTolerance);
        lowPid.setTolerance(ElevatorConstants.BaseGoalTolerance);
        if (RuntimeConstants.TuningMode) {
            ElevatorCharacterization.enable(this);
            TuningModeTab.getInstance().addCommand("Run to 0",
                runOnce(() -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MinHeight)));
            TuningModeTab.getInstance().addCommand("Run to max",
                runOnce(() -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MaxHeight)));
            TuningModeTab.getInstance().addCommand("Toggle elevator coast", runOnce(() -> {
                if (coasting) {
                    io.setCoast(false);
                    coasting = false;
                } else {
                    io.setCoast(true);
                    coasting = true;
                }
            }));
        }
    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double setGoal) {
        if (setGoal > ElevatorConstants.MaxHeight || setGoal < ElevatorConstants.MinHeight) return;
        this.goalInches = setGoal;
        lowPid.setSetpoint(setGoal);
        pid.setSetpoint(setGoal);
    }

    public boolean atGoal() {
        if (lastMeasurement > stopMap.get()) {
            return pid.atSetpoint();
        } else {
            return lowPid.atSetpoint();
        }
    }

    public double getMeasurement() { return lastMeasurement; }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = lastMeasurement < stopMap.get() ? constant.get() * lastMeasurement : kg.get();

        double voltOut = output + feedforward +
            Trig.cosizzle(Arm.getInstance().getMeasurement()) * ElevatorConstants.kGForArm.get();

        io.setVoltage(voltOut);
    }

    public TuneablePIDController getPid() { return lastMeasurement > stopMap.get() ? pid : lowPid; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        lastMeasurement = Algebra.linearMap(inputs.wirePotentiometer, ElevatorConstants.PotentiometerMinVolt,
            ElevatorConstants.PotentiometerMaxVolt, ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight);
        differentiableMeasurementInches.update(lastMeasurement);

        if (!hasResetPidController) {
            // pid.reset(new TrapezoidProfile.State(getMeasurement().in(Inches), 0));
            hasResetPidController = true;
        }
        if (lastMeasurement > stopMap.get() && pid.getError() > 0) {
            useOutput(pid.calculate(lastMeasurement), new TrapezoidProfile.State(pid.getSetpoint(), 0));
        } else {
            useOutput(lowPid.calculate(lastMeasurement), new TrapezoidProfile.State(pid.getSetpoint(), 0));
        }

        Logger.recordOutput("Elevator/Actual", lastMeasurement);
        Logger.recordOutput("Elevator/Goal", goalInches);
        Logger.recordOutput("Elevator/Error", pid.getError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
