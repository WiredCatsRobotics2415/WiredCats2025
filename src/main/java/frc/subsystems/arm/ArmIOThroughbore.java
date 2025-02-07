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

public class ArmIOThroughbore implements ArmIO {
    private TalonFX left = new TalonFX(ArmConstants.LeftMotorID);
    private TalonFX right = new TalonFX(ArmConstants.RightMotorID);
    private double appliedVoltage;

    private StatusSignal<Current> leftStator = left.getStatorCurrent();
    private StatusSignal<Current> leftSupply = left.getSupplyCurrent();
    private StatusSignal<Temperature> leftTemp = left.getDeviceTemp();

    private StatusSignal<Current> rightStator = right.getStatorCurrent();
    private StatusSignal<Current> rightSupply = right.getSupplyCurrent();
    private StatusSignal<Temperature> rightTemp = right.getDeviceTemp();

    private DutyCycleEncoder throughbore;

    public ArmIOThroughbore() {
        //I'm not sure this is correct
        throughbore = new DutyCycleEncoder(ArmConstants.ThroughborePort, ArmConstants.MaxDegreesFront - ArmConstants.MaxDegreesBack, (ArmConstants.MaxDegreesFront - ArmConstants.MaxDegreesBack)/2);

        configureMotors();
    }

    public void configureMotors() {
        left = new TalonFX(ArmConstants.LeftMotorID);
        left.getConfigurator().apply(ArmConstants.MotorOutput);

        right = new TalonFX(ArmConstants.RightMotorID);
        right.setControl(new StrictFollower(left.getDeviceID()));
        right.getConfigurator().apply(ArmConstants.MotorOutput);

        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
        BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, leftTemp,
            rightStator, rightSupply, rightTemp);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(leftStator, leftSupply, leftTemp, rightStator,
            rightSupply, rightTemp);

        inputs.leftConnected = left.isAlive();
        inputs.leftStatorCurrent = leftStator.getValue();
        inputs.leftSupplyCurrent = leftSupply.getValue();
        inputs.leftTemp = leftTemp.getValue();

        inputs.rightConnected = right.isAlive();
        inputs.rightStatorCurrent = rightStator.getValue();
        inputs.rightSupplyCurrent = rightSupply.getValue();
        inputs.rightTemp = rightTemp.getValue();

        inputs.appliedVoltage = appliedVoltage;
        //I'm not sure this is correct
        inputs.position = throughbore.get();
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        left.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean coast) {
        left.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        right.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}