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
    @Getter private TuneablePIDController pid = new TuneablePIDController(ElevatorConstants.kP, 0.0d,
        ElevatorConstants.kD, "ElevatorPID");

    private TuneableNumber downThreshold = new TuneableNumber(3, "Elevator/downThreshold");
    private TuneableNumber downPower = new TuneableNumber(0.1, "Elevator/downPower");

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, new ElevatorIO() {});
        pid.setTolerance(ElevatorConstants.BaseGoalTolerance);
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
        pid.setSetpoint(setGoal.in(Inches));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public Distance getMeasurement() { return lastMeasurement; }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.velocity);
        double voltOut = output + feedforward +
            Trig.cosizzle(Arm.getInstance().getMeasurement()) * ElevatorConstants.kGForArm.get();

        io.setVoltage(voltOut);
    }

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
        useOutput(pid.calculate(measurementInches), new TrapezoidProfile.State(pid.getSetpoint(), 0));

        Logger.recordOutput("Elevator/Actual", measurementInches);
        Logger.recordOutput("Elevator/Goal", goal);
        Logger.recordOutput("Elevator/Error", pid.getPositionError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
