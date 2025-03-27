package frc.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.constants.Subsystems.ClimberConstants;

public class ClimbIOReal implements ClimbIO {
    private TalonFX motor;

    private StatusSignal<Current> motorStator;
    private StatusSignal<Current> motorSupply;
    private StatusSignal<Temperature> motorTemp;

    public ClimbIOReal() {
        configureMotor();
    }

    private void configureMotor() {
        motor = new TalonFX(ClimberConstants.MotorID);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        motor.setNeutralMode(NeutralModeValue.Brake);

        motorStator = motor.getStatorCurrent();
        motorSupply = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStator, motorSupply, motorTemp, motor.getRotorPosition());
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorConnected = motor.isAlive();
        inputs.motorStatorCurrent = motorStator.getValueAsDouble();
        inputs.motorSupplyCurrent = motorSupply.getValueAsDouble();
        inputs.motorTemp = motorTemp.getValueAsDouble();
        inputs.motorRotations = motor.getRotorPosition().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
