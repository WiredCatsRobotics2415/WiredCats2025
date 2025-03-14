package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Subsystems.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private double appliedVoltage;

    private StatusSignal<Current> leftStator;
    private StatusSignal<Current> leftSupply;
    private StatusSignal<Temperature> leftTemp;

    private StatusSignal<Current> rightStator;
    private StatusSignal<Current> rightSupply;
    private StatusSignal<Temperature> rightTemp;

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

        rightStator = rightMotor.getStatorCurrent();
        rightSupply = rightMotor.getSupplyCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, leftTemp, rightStator, rightSupply,
            rightTemp);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isConnectedLeft = leftMotor.isAlive();
        inputs.temperatureLeft = leftTemp.getValue();
        inputs.statorCurrentLeft = leftStator.getValue();
        inputs.supplyCurrentLeft = leftSupply.getValue();

        inputs.isConnectedRight = rightMotor.isAlive();
        inputs.temperatureRight = rightTemp.getValue();
        inputs.statorCurrentRight = rightStator.getValue();
        inputs.supplyCurrentRight = rightSupply.getValue();

        inputs.appliedVoltage = Volts.of(appliedVoltage);
        inputs.wirePotentiometer = wirePotentiometer.getVoltage();
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
