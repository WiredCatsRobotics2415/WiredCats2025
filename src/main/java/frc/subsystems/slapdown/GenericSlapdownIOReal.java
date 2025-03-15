package frc.subsystems.slapdown;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.utils.hardware.Throughbore;

public class GenericSlapdownIOReal implements GenericSlapdownIO {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private Throughbore throughbore;
    private AnalogInput sensor;

    private double appliedVolts;

    public GenericSlapdownIOReal() {}

    @Override
    public void updateInputs(GenericSlapdownIOInputsAutoLogged inputs) {
        if (pivotMotor == null || intakeMotor == null) return;

        inputs.pivotConnected = !pivotMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.pivotTemp = pivotMotor.getMotorTemperature();
        inputs.pivotStatorCurrent = pivotMotor.getOutputCurrent();
        inputs.appliedVoltage = appliedVolts;

        inputs.intakeConnected = !intakeMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.intakeTemp = intakeMotor.getMotorTemperature();
        inputs.intakeStatorCurrent = intakeMotor.getOutputCurrent();

        throughbore.get();
        inputs.throughborePosition = throughbore.getRaw();
        if (sensor != null) inputs.sensorValue = sensor.getValue();
    }

    @Override
    public void configureHardware(int pivotId, int intakeId, int throughborePort, double tBorMin, double tBorMax,
        boolean tBorWrap, int sensorAnalogPort) {
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20, 40).idleMode(IdleMode.kBrake)
            .voltageCompensation(12).inverted(false);

        pivotMotor = new SparkMax(pivotId, MotorType.kBrushless);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.setCANTimeout(250);

        intakeMotor = new SparkMax(intakeId, MotorType.kBrushless);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.setCANTimeout(250);

        throughbore = new Throughbore(throughborePort, tBorMin, tBorMax, tBorWrap, "GenericSlapdown/tBor");
        if (sensorAnalogPort > 0) sensor = new AnalogInput(sensorAnalogPort);
    }

    @Override
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        appliedVolts = voltage;
        pivotMotor.setVoltage(voltage);
    }
}
