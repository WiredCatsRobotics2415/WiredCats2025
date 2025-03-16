package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
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
    @Getter private Distance goal = Inches.of(0.0);
    private Distance lastMeasurement = Inches.of(0.0);
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();
    private boolean hasResetPidController = false;

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
            TuningModeTab.getInstance().addCommand("Run to 0", runOnce(
                () -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MinHeight.distance())));
            TuningModeTab.getInstance().addCommand("Run to max", runOnce(
                () -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MaxHeight.distance())));
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
    public void setGoal(Distance setGoal) {
        if (setGoal.gt(ElevatorConstants.MaxHeight.distance()) || setGoal.lt(ElevatorConstants.MinHeight.distance()))
            return;
        this.goal = setGoal;
        lowPid.setSetpoint(setGoal.in(Inches));
        pid.setSetpoint(setGoal.in(Inches));
    }

    public boolean atGoal() {
        if (lastMeasurement.gte(stopMap.distance())) {
            return pid.atSetpoint();
        } else {
            return lowPid.atSetpoint();
        }
    }

    public Distance getMeasurement() { return lastMeasurement; }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = lastMeasurement.lte(stopMap.distance()) ? constant.get() * lastMeasurement.in(Inches)
            : kg.get();

        double voltOut = output + feedforward +
            Trig.cosizzle(Arm.getInstance().getMeasurement()) * ElevatorConstants.kGForArm.get();

        io.setVoltage(voltOut);
    }

    public TuneablePIDController getPid() { return lastMeasurement.gte(stopMap.distance()) ? pid : lowPid; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        double measurementInches = Algebra.linearMap(inputs.wirePotentiometer,
            ElevatorConstants.PotentiometerMinVolt.get(), ElevatorConstants.PotentiometerMaxVolt.get(),
            ElevatorConstants.MinHeight.get(), ElevatorConstants.MaxHeight.get());
        differentiableMeasurementInches.update(measurementInches);
        lastMeasurement = Inches.of(measurementInches);

        if (!hasResetPidController) {
            // pid.reset(new TrapezoidProfile.State(getMeasurement().in(Inches), 0));
            hasResetPidController = true;
        }
        if (lastMeasurement.gte(stopMap.distance()) && pid.getError() > 0) {
            useOutput(pid.calculate(measurementInches), new TrapezoidProfile.State(pid.getSetpoint(), 0));
        } else {
            useOutput(lowPid.calculate(measurementInches), new TrapezoidProfile.State(pid.getSetpoint(), 0));
        }

        Logger.recordOutput("Elevator/Actual", measurementInches);
        Logger.recordOutput("Elevator/Goal", goal);
        Logger.recordOutput("Elevator/Error", pid.getError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
