package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private boolean coasting;
    @Getter
    @AutoLogOutput(key = "Elevator/Goal") private Distance goal = Inches.of(0.0);
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();

    private ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.kP, 0, ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(ElevatorConstants.VelocityMax.get(), ElevatorConstants.AccelerationMax.get()));

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, new ElevatorIO() {});
        if (RuntimeConstants.TuningMode) {
            ElevatorCharacterization.enable(this);
            ElevatorConstants.GoalTolerance.addListener(() -> pid.setTolerance(ElevatorConstants.GoalTolerance.get()));
            ElevatorConstants.VelocityMax.addListener(
                () -> pid.setConstraints(new TrapezoidProfile.Constraints(ElevatorConstants.VelocityMax.get(),
                    pid.getConstraints().maxAcceleration)));
            ElevatorConstants.AccelerationMax
                .addListener(() -> pid.setConstraints(new TrapezoidProfile.Constraints(pid.getConstraints().maxVelocity,
                    ElevatorConstants.AccelerationMax.get())));
            TuningModeTab.getInstance().addCommand("Run to 0", runOnce(() -> setGoal(ElevatorConstants.MinHeight)));
            TuningModeTab.getInstance().addCommand("Run to max", runOnce(() -> setGoal(ElevatorConstants.MaxHeight)));
            TuningModeTab.getInstance().addCommand("Toggle coast", runOnce(() -> {
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
        if (setGoal.gt(ElevatorConstants.MaxHeight) || setGoal.lt(ElevatorConstants.MinHeight)) return;
        this.goal = setGoal;
        pid.setGoal(setGoal.in(Inches));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public Distance getMeasurement() {
        return Inches.of(Algebra.linearMap(inputs.wirePotentiometer, ElevatorConstants.PotentiometerMinVolt.in(Volts),
            ElevatorConstants.PotentiometerMaxVolt.in(Volts), ElevatorConstants.MinHeight.in(Inches),
            ElevatorConstants.MaxHeight.in(Inches)));
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position);
        double voltOut = output + feedforward;
        io.setVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        double measurementInches = getMeasurement().in(Inches);
        differentiableMeasurementInches.update(measurementInches);
        useOutput(pid.calculate(measurementInches), pid.getSetpoint());

        Logger.recordOutput("Elevator/Error", pid.getPositionError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
