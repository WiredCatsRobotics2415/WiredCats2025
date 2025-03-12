package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOReal implements ArmIO {
    private TalonFX motor;
    private double appliedVoltage;

    private StatusSignal<Current> motorStator;
    private StatusSignal<Current> motorSupply;
    private StatusSignal<Temperature> motorTemp;

    private DutyCycleEncoder throughbore;

    public ArmIOReal() {
        throughbore = new DutyCycleEncoder(ArmConstants.ThroughborePort);

        configureMotors();
    }

    public void configureMotors() {
        motor = new TalonFX(ArmConstants.MotorID);
        motor.getConfigurator().apply(ArmConstants.MotorOutput);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motorStator = motor.getStatorCurrent();
        motorSupply = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStator, motorSupply, motorTemp);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorStator, motorSupply, motorTemp);

        inputs.motorConnected = motor.isAlive();
        inputs.motorStatorCurrent = motorStator.getValue();
        inputs.motorSupplyCurrent = motorSupply.getValue();
        inputs.motorTemp = motorTemp.getValue();

        inputs.appliedVoltage = Volts.of(appliedVoltage);
        inputs.throughborePosition = throughbore.get();
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        motor.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
