package frc.subsystems.algaeintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.constants.Subsystems.AlgaeIntakeConstants;
import frc.utils.io.RealIO;

public class AlgaeIntakeIOReal extends RealIO implements AlgaeIntakeIO {
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
        //set to counter clockwise positive
        motor.getConfigurator().apply(AlgaeIntakeConstants.MotorOutput);
        motor.getConfigurator().apply(AlgaeIntakeConstants.CurrentLimit);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent, motorSupplyCurrent, motorTemp);
        // Is this right? Manually convert the TalonFX motor to a MotorController type to avoid the error...
        registerMotors((MotorController)motor);
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
        if (!areMotorsEnabled()) return;
        motor.set(speed);
    }

    @Override
    public void off() {
        if (!areMotorsEnabled()) return;
        motor.set(0);
    }
}
