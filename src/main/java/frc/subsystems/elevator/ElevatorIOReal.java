package frc.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.constants.Subsystems.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private double appliedVoltage;

    private StatusSignal<Current> leftStator;
    private StatusSignal<Current> leftSupply;
    private StatusSignal<Temperature> leftTemp;
    private StatusSignal<Angle> leftRotorPosition;
    private StatusSignal<AngularVelocity> leftRotorVelocity;

    private StatusSignal<Current> rightStator;
    private StatusSignal<Current> rightSupply;
    private StatusSignal<Temperature> rightTemp;

    private double rotorRotationToElevatorHeight = (1.7567 * 3 * Math.PI) / 5.0d;

    private AnalogInput wirePotentiometer;

    public ElevatorIOReal() {
        wirePotentiometer = new AnalogInput(ElevatorConstants.AnalogPotentiometerPort);
        configureMotors();
    }

    private void configureMotors() {
        leftMotor = new TalonFX(ElevatorConstants.LeftMotorID);
        leftMotor.getConfigurator()
            .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        rightMotor = new TalonFX(ElevatorConstants.RightMotorID);
        rightMotor.getConfigurator()
            .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftStator = leftMotor.getStatorCurrent();
        leftSupply = leftMotor.getSupplyCurrent();
        leftTemp = leftMotor.getDeviceTemp();
        leftRotorPosition = leftMotor.getRotorPosition();
        leftRotorVelocity = leftMotor.getRotorVelocity();

        rightStator = rightMotor.getStatorCurrent();
        rightSupply = rightMotor.getSupplyCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, leftTemp, rightStator, rightSupply,
            rightTemp, leftRotorPosition, leftRotorVelocity);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(leftTemp, leftStator, leftSupply, rightTemp, rightStator, rightSupply);

        inputs.isConnectedLeft = leftMotor.isAlive();
        inputs.temperatureLeft = leftTemp.getValueAsDouble();
        inputs.statorCurrentLeft = leftStator.getValueAsDouble();
        inputs.supplyCurrentLeft = leftSupply.getValueAsDouble();

        inputs.isConnectedRight = rightMotor.isAlive();
        inputs.temperatureRight = rightTemp.getValueAsDouble();
        inputs.statorCurrentRight = rightStator.getValueAsDouble();
        inputs.supplyCurrentRight = rightSupply.getValueAsDouble();

        inputs.appliedVoltage = appliedVoltage;
        inputs.wirePotentiometer = wirePotentiometer.getVoltage() / RobotController.getVoltage5V();
        inputs.inches = (inputs.wirePotentiometer * 241.25) - 16.93;
        // Unstandardized:
        // inputs.inches = Math.pow(19.25064 * wirePotentiometer.getVoltage(), 2) + (-8.32661 * wirePotentiometer.getVoltage()) +
        // 1.56748;

        // Standardized:
        // inputs.inches = (481.26598 * Math.pow(inputs.wirePotentiometer, 2)) + (-41.63306 * inputs.wirePotentiometer) +
        // 0.46748;

        // Rotor encoder:
        // inputs.inches = BaseStatusSignal.getLatencyCompensatedValueAsDouble(leftRotorPosition, leftRotorVelocity) * rotorRotationToElevatorHeight;
    }

    @Override
    public void setVoltage(double voltOut) {
        appliedVoltage = voltOut;
        leftMotor.setVoltage(voltOut);
    }

    @Override
    public void setCoast(boolean coast) {
        if (coast) {
            leftMotor.setNeutralMode(NeutralModeValue.Coast);
            rightMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            leftMotor.setNeutralMode(NeutralModeValue.Brake);
            rightMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }
}
