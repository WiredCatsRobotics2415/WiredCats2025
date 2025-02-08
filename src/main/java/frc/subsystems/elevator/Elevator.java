package frc.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Util;
import lombok.Getter;

public class Elevator extends SubsystemBase {
    @Getter private double goalInches = 0.0;

    private ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.kP, 0, ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(ElevatorConstants.VelocityMax, ElevatorConstants.AccelerationMax));

    private ElevatorIO io;
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
    public void setGoal(double goalInches) {
        if (goalInches > ElevatorConstants.MaxHeightInches || goalInches < ElevatorConstants.MinHeightInches) return;
        this.goalInches = goalInches;
        pid.setGoal(goalInches);
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
        useOutput(pid.calculate(measurement), pid.getSetpoint());
    }
}
