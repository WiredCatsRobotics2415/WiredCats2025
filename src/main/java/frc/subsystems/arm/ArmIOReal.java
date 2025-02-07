package frc.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.constants.Subsystems;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOReal implements ArmIO {
    private TalonFX motor = new TalonFX(ArmConstants.MotorID);
    private double appliedVoltage;

    private StatusSignal<Current> motorStator = motor.getStatorCurrent();
    private StatusSignal<Current> motorSupply = motor.getSupplyCurrent();
    private StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();

    private DutyCycleEncoder throughbore;

    public ArmIOReal() {
        //I'm not sure this is correct
        throughbore = new DutyCycleEncoder(ArmConstants.ThroughborePort, ArmConstants.MaxDegreesFront - ArmConstants.MaxDegreesBack, (ArmConstants.MaxDegreesFront - ArmConstants.MaxDegreesBack)/2);

        configureMotors();
    }

    public void configureMotors() {
        motor = new TalonFX(ArmConstants.MotorID);
        motor.getConfigurator().apply(ArmConstants.MotorOutput);

        motor.setNeutralMode(NeutralModeValue.Brake);
        
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStator, motorSupply, motorTemp);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStator, motorSupply, motorTemp);

        inputs.motorConnected = motor.isAlive();
        inputs.motorStatorCurrent = motorStator.getValue();
        inputs.motorSupplyCurrent = motorSupply.getValue();
        inputs.motorTemp = motorTemp.getValue();

        inputs.appliedVoltage = appliedVoltage;
        //I'm not sure this is correct
        inputs.position = throughbore.get();
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        motor.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}