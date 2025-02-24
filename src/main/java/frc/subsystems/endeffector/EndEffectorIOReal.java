package frc.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

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

    public EndEffectorIOReal() {
        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config.smartCurrentLimit(20, 40).idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.sensorValue = irSensor.getValue();

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = Amps.of(motor.getOutputCurrent());
        inputs.motorTemp = Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }
}
