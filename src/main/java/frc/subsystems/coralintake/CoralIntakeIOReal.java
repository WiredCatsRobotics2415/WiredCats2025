package frc.subsystems.coralintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.constants.Subsystems.CoralIntakeConstants;

public class CoralIntakeIOReal implements CoralIntakeIO {
    private TalonFX motor = new TalonFX(CoralIntakeConstants.MotorID);

    private StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent();
    private StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent();
    private StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();

    public CoralIntakeIOReal() {
        configureMotor();
    }

    private void configureMotor() {
        motor.setNeutralMode(NeutralModeValue.Brake);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent, motorSupplyCurrent, motorTemp);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorSupplyCurrent, motorTemp);

        inputs.motorConnected = motor.isConnected();
        inputs.motorStatorCurrent = motorStatorCurrent.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorTemp = motorTemp.getValue();
    }

    @Override
    public void setPower(double speed) {
        motor.set(speed);
    }
}
