package frc.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Utils;
import lombok.Getter;
// from using AnalogPotentiometer before
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.StrictFollower;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;

public class Elevator extends SubsystemBase {

    @Getter private double goalInches = 0.0;

    private static Elevator instance;

    private AnalogInput input; 

    private static boolean isCoasting = false;
    private ElevatorIO io;

    private ElevatorFeedforward ff =
            new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

    private ProfiledPIDController pid =
            new ProfiledPIDController(
                    ElevatorConstants.KP,
                    0,
                    ElevatorConstants.KD,
                    new TrapezoidProfile.Constraints(
                            ElevatorConstants.VELO_MAX, ElevatorConstants.ACCEL_MAX));


    private Elevator() {

        io = (ElevatorIO) Utils.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, ElevatorIO.class);

        input = new AnalogInput(ElevatorConstants.ANALOG_POT_PORT); 

        setGoal(getMeasurement()); 
    }

    // get current potentiometer angle
    public double getPotentiometerAngle(AnalogPotentiometer pot) {
        return pot.get(); 
    }

    public double getMeasurement() {
        double rotations = 0.0d; 
        rotations = getPotRotations(); 
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
        if (!isCoasting) {
            io.setVoltage(voltOut); 
        }
        SmartDashboard.putNumber("Elevator Volt out", voltOut); 
    }

    private double getPotRotations() {
        // from last years code
        // !!!!!! elevator height may not correlate to potentiometer angle !!!!!
        double measure = ElevatorConstants.MaxHeight - (((input.getAverageVoltage() - ElevatorConstants.MIN_VOLT) / (ElevatorConstants.MAX_VOLT - ElevatorConstants.MIN_VOLT))
        * ElevatorConstants.MaxHeight); 
        return measure; 
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalInches) {
        if (goalInches > ElevatorConstants.MaxHeight || goalInches < 0) return;
        this.goalInches = goalInches;
        // do I need to convert from goalInches to goal degrees?
        pid.setGoal(new TrapezoidProfile.State(goalInches, 0)); 
    }

    public boolean atGoal() {
        return true;
    }

    public InstantCommand changeCoast = new InstantCommand(() -> {
        if (isCoasting == false) {
            isCoasting = true;
        } else {
            isCoasting = false;
        }
    });

    @Override
    public void periodic() {
        double measurement = getMeasurement();
        SmartDashboard.putNumber("Elevator Measurement", measurement);
        SmartDashboard.putNumber("Elevator Voltage", input.getAverageVoltage()); 
        SmartDashboard.putData("coast", changeCoast);
        useOutput(pid.calculate(measurement), pid.getSetpoint());

        SmartDashboard.putNumber("goalindegrees", goalInches);
    }
}