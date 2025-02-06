package frc.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Utils;
import lombok.Getter;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs;
    private static Elevator instance;

    @Getter private double goalInches = 0.0;

    private ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG,
        ElevatorConstants.KV, ElevatorConstants.KA);
    private ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.KP, 0, ElevatorConstants.KD,
        new TrapezoidProfile.Constraints(ElevatorConstants.VELO_MAX, ElevatorConstants.ACCEL_MAX));

    private Elevator() {
        io = (ElevatorIO) Utils.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, ElevatorIO.class);

        setGoal(getMeasurement());
    }

    public double getMeasurement() {
        double rotations = 0.0d;
        return rotations;
    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
        double voltOut = output + feedforward;
        // set motors based on voltOut
        io.setVoltage(voltOut);
        SmartDashboard.putNumber("Elevator Volt out", voltOut);
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalInches) {
        if (goalInches > ElevatorConstants.MaxHeight || goalInches < 0) return;
        this.goalInches = goalInches;
        pid.setGoal(new TrapezoidProfile.State(goalInches, 0));
    }

    public boolean atGoal() {
        return true;
    }

    @Override
    public void periodic() {
        double measurement = getMeasurement();
        SmartDashboard.putNumber("Elevator Measurement", measurement);
        useOutput(pid.calculate(measurement), pid.getSetpoint());

        SmartDashboard.putNumber("goalindegrees", goalInches);
    }
}
