package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class GenericSlapdownIOReal implements GenericSlapdownIO {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private DutyCycleEncoder throughbore;
    private AnalogInput sensor;

    public GenericSlapdownIOReal() {}

    @Override
    public void updateInputs(GenericSlapdownIOInputsAutoLogged inputs) {
        if (pivotMotor == null || intakeMotor == null) return;

        inputs.pivotConnected = pivotMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.pivotTemp = Celsius.of(pivotMotor.getMotorTemperature());
        inputs.pivotStatorCurrent = Amps.of(pivotMotor.getOutputCurrent());

        inputs.intakeConnected = intakeMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.intakeTemp = Celsius.of(intakeMotor.getMotorTemperature());
        inputs.intakeStatorCurrent = Amps.of(intakeMotor.getOutputCurrent());

        inputs.throughborePosition = throughbore.get();
        if (sensor != null) inputs.sensorValue = sensor.getValue();
    }

    @Override
    public void configureHardware(int pivotId, int intakeId, int throughborePort, int sensorAnalogPort) {
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20, 40).idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        pivotMotor = new SparkMax(pivotId, MotorType.kBrushless);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.setCANTimeout(250);

        intakeMotor = new SparkMax(intakeId, MotorType.kBrushless);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.setCANTimeout(250);

        throughbore = new DutyCycleEncoder(throughborePort);
        if (sensorAnalogPort > 0) sensor = new AnalogInput(sensorAnalogPort);
    }

    @Override
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }
}
