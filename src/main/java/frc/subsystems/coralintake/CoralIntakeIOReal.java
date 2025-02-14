package frc.subsystems.coralintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Subsystems;
import frc.constants.Subsystems.CoralIntakeConstants;

public class CoralIntakeIOReal implements CoralIntakeIO {
    private TalonFX motor = new TalonFX(CoralIntakeConstants.MotorID);
    private AnalogInput irSensor;

    private StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent();
    private StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent();
    private StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();

    public CoralIntakeIOReal() {
        configureMotor();

        irSensor = new AnalogInput(CoralIntakeConstants.IRSensor);
    }

    private void configureMotor() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        // set to counter clockwise positive
        motor.getConfigurator().apply(CoralIntakeConstants.MotorOutput);
        motor.getConfigurator().apply(CoralIntakeConstants.CurrentLimit);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent, motorSupplyCurrent, motorTemp);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorSupplyCurrent, motorTemp);
        inputs.sensorValue = irSensor.getValue();

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = motorStatorCurrent.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorTemp = motorTemp.getValue();
    }

    @Override
    public void setPower(double speed) {
        motor.set(speed);
    }

    @Override
    public void off() {
        motor.set(0);
    }
}
