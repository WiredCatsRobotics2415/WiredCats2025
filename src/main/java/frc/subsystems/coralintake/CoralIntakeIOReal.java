package frc.subsystems.coralintake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.constants.Subsystems.CoralIntakeConstants;

public class CoralIntakeIOReal implements CoralIntakeIO {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private DutyCycleEncoder throughbore;

    private double appliedVolts;
    private double appliedIntakeVolts;

    public CoralIntakeIOReal() {
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(30, 60).idleMode(IdleMode.kBrake)
            .voltageCompensation(12).inverted(true);

        pivotMotor = new SparkMax(CoralIntakeConstants.PivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.setCANTimeout(250);

        intakeMotor = new SparkMax(CoralIntakeConstants.IntakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.setCANTimeout(250);

        throughbore = new DutyCycleEncoder(CoralIntakeConstants.ThroughborePort);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        if (pivotMotor == null || intakeMotor == null) return;

        inputs.pivotConnected = !pivotMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.pivotTemp = pivotMotor.getMotorTemperature();
        inputs.pivotStatorCurrent = pivotMotor.getOutputCurrent();
        inputs.appliedVoltage = appliedVolts;

        inputs.intakeConnected = !intakeMotor.getLastError().equals(REVLibError.kCANDisconnected);
        inputs.intakeTemp = intakeMotor.getMotorTemperature();
        inputs.intakeStatorCurrent = intakeMotor.getOutputCurrent();
        inputs.appliedIntakeVolts = appliedIntakeVolts;

        double tBor = throughbore.get();
        if (tBor < 0.8 && tBor > 0) {
            tBor += 1;
        }
        inputs.throughborePosition = tBor;
    }

    @Override
    public void setIntakeVoltage(double volts) {
        appliedIntakeVolts = volts;
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        appliedVolts = voltage;
        pivotMotor.setVoltage(voltage);
    }
}
