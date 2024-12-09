package frc.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Subsystems;
import frc.constants.Subsystems.IntakeConstants;
import frc.util.io.RealIO;

public class IntakeIOReal extends RealIO implements IntakeIO {
    private TalonFX motor = new TalonFX(IntakeConstants.IntakeMotorID);
    private AnalogInput closeToFlywheelSensor;

    private StatusSignal<Double> motorStatorCurrent = motor.getStatorCurrent();
    private StatusSignal<Double> motorSupplyCurrent = motor.getSupplyCurrent();
    private StatusSignal<Double> motorTemp = motor.getDeviceTemp();

    public IntakeIOReal() {
        configureMotor();

        closeToFlywheelSensor = new AnalogInput(IntakeConstants.FlywheelIR);
    }

    private void configureMotor() {
        motor.setInverted(Subsystems.TalonFXDirectionCounterClockWise);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(IntakeConstants.CurrentLimit);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent,
            motorSupplyCurrent, motorTemp);
        registerMotors(motor);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorSupplyCurrent, motorTemp);
        inputs.sensorTrigger = closeToFlywheelSensor
            .getValue() < Subsystems.IntakeConstants.IRThreshold;

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = motorStatorCurrent.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorTemp = motorTemp.getValue();
    }

    @Override
    public void on(double speed) {
        if (!areMotorsEnabled()) return;
        motor.set(speed);
    }

    @Override
    public void off() {
        if (!areMotorsEnabled()) return;
        motor.set(0);
    }
}
