package frc.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
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

    private ElevatorFeedforward ff =
            new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

    private ProfiledPIDController pid =
            new ProfiledPIDController(
                    ElevatorConstants.KP,
                    0,
                    ElevatorConstants.KD,
                    new TrapezoidProfile.Constraints(
                            ElevatorConstants.VELO_MAX, ElevatorConstants.ACCEL_MAX));

    
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private Elevator() {
        input = new AnalogInput(ElevatorConstants.ANALOG_POT_PORT); 

        configureMotors(); 

        setGoal(getMeasurement()); 
    }

    // get current potentiometer angle
    public double getPotentiometerAngle(AnalogPotentiometer pot) {
        return pot.get(); 
    }

    public void configureMotors() {
        // two talon FXs, neither inverted
        FeedbackConfigs feedbackConfigs =
                new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_PORT); 
        leftMotor.getConfigurator().apply(feedbackConfigs); 
        
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_PORT); 
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID())); 
        
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
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

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity); 
        double voltOut = output + feedforward; 
        // set motors based on voltOut
        if (!isCoasting) {
            leftMotor.setVoltage(voltOut); 
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

    public void brake() {
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
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