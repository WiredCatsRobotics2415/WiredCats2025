package frc.subsystems.endeffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Subsystems.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
    private SparkMax motor = new SparkMax(EndEffectorConstants.MotorID, MotorType.kBrushless);
    private AnalogInput irSensor = new AnalogInput(EndEffectorConstants.IRSensorPort);
    double appliedVolts;

    public EndEffectorIOReal() {
        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config.smartCurrentLimit(20, 40).idleMode(IdleMode.kBrake).inverted(true),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.sensorValue = (int) irSensor.getValue();

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        motor.set(volts);
    }
}
