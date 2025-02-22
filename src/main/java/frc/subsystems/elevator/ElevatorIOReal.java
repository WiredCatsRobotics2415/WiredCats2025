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

    private StatusSignal<Current> leftStator = leftMotor.getStatorCurrent();
    private StatusSignal<Current> leftSupply = leftMotor.getSupplyCurrent();
    private StatusSignal<Temperature> leftTemp = leftMotor.getDeviceTemp();

    private StatusSignal<Current> rightStator = rightMotor.getStatorCurrent();
    private StatusSignal<Current> rightSupply = rightMotor.getSupplyCurrent();
    private StatusSignal<Temperature> rightTemp = rightMotor.getDeviceTemp();

    private AnalogInput wirePotentiometer;

    public ElevatorIOReal() {
        wirePotentiometer = new AnalogInput(ElevatorConstants.AnalogPotentiometerPort);
        configureMotors();
    }

    private void configureMotors() {
        leftMotor = new TalonFX(ElevatorConstants.LeftMotorPort);
        leftMotor.getConfigurator()
            .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        rightMotor = new TalonFX(ElevatorConstants.RightMotorPort);
        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

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

    public void setVoltage(double voltOut) {
        appliedVoltage = voltOut;
        leftMotor.setVoltage(voltOut);
    }
}
