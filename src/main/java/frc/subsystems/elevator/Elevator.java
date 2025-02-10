package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.DoubleDifferentiableValue;
import frc.utils.Util;
import lombok.Getter;

public class Elevator extends SubsystemBase {
    @Getter private Distance goal = Inches.of(0.0);
    @Getter private DoubleDifferentiableValue measurementInches = new DoubleDifferentiableValue();

    private ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.kP, 0, ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(ElevatorConstants.VelocityMax, ElevatorConstants.AccelerationMax));

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, ElevatorIO.class);
    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(Distance setGoal) {
        if (setGoal.in(Inches) > ElevatorConstants.MaxHeightInches
            || setGoal.in(Inches) < ElevatorConstants.MinHeightInches) return;
        this.goal = setGoal;
        pid.setGoal(setGoal.in(Inches));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public double getMeasurement() {
        return Util.linearMap(inputs.wirePotentiometerValue, ElevatorConstants.PotentiometerMinVolt,
            ElevatorConstants.PotentiometerMaxVolt, ElevatorConstants.MinHeightInches,
            ElevatorConstants.MaxHeightInches);
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position);
        double voltOut = output + feedforward;
        io.setVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double measurement = getMeasurement();
        measurementInches.update(measurement);
        useOutput(pid.calculate(measurement), pid.getSetpoint());
    }
}
