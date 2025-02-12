package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.DoubleDifferentiableValue;
import frc.utils.Util;
import lombok.Getter;

public class Elevator extends SubsystemBase {
    @Getter private Distance goal = Inches.of(0.0);
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();

    private ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.kP, 0, ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(ElevatorConstants.VelocityMax, ElevatorConstants.AccelerationMax));

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        pid.setTolerance(ArmConstants.GoalTolerance);
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, ElevatorIO.class);
        if (RuntimeConstants.TuningMode) {
            ElevatorCharacterization.enable(this);
        }
    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(Distance setGoal) {
        if (setGoal.gt(ElevatorConstants.MaxHeightInches) || setGoal.lt(ElevatorConstants.MinHeightInches)) return;
        this.goal = setGoal;
        pid.setGoal(setGoal.in(Inches));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public Distance getMeasurement() {
        return Inches.of(Util.linearMap(inputs.wirePotentiometer, ElevatorConstants.PotentiometerMinVolt.in(Volts),
            ElevatorConstants.PotentiometerMaxVolt.in(Volts), ElevatorConstants.MinHeightInches.in(Inches),
            ElevatorConstants.MaxHeightInches.in(Inches)));
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position);
        double voltOut = output + feedforward;
        io.setVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double measurementInches = getMeasurement().in(Inches);
        differentiableMeasurementInches.update(measurementInches);
        useOutput(pid.calculate(measurementInches), pid.getSetpoint());
    }
}
