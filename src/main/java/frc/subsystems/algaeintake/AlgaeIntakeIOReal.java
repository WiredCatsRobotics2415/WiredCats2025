package frc.subsystems.algaeintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Subsystems.AlgaeIntakeConstants;

public class AlgaeIntakeIOReal implements AlgaeIntakeIO {
    private TalonFX motor = new TalonFX(AlgaeIntakeConstants.AlgaeIntakeMotorID);
    private DigitalInput limitSwitchInput;

    private StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent();
    private StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent();
    private StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();

    public AlgaeIntakeIOReal() {
        configureMotor();

        limitSwitchInput = new DigitalInput(AlgaeIntakeConstants.LimitSwitchID);
    }

    private void configureMotor() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        // set to counter clockwise positive
        motor.getConfigurator().apply(AlgaeIntakeConstants.MotorOutput);
        motor.getConfigurator().apply(AlgaeIntakeConstants.CurrentLimit);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent, motorSupplyCurrent, motorTemp);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorSupplyCurrent, motorTemp);
        inputs.limitSwitch = limitSwitchInput.get();

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
