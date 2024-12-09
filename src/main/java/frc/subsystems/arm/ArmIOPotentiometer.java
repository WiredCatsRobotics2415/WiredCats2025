package frc.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Subsystems;
import frc.constants.Subsystems.ArmConstants;
import frc.util.hardware.REV10KPotentiometer;
import frc.util.io.RealIO;

public class ArmIOPotentiometer extends RealIO implements ArmIO {
    private TalonFX left = new TalonFX(ArmConstants.LeftMotorID);
    private TalonFX right = new TalonFX(ArmConstants.RightMotorID);
    private double appliedVoltage;

    private StatusSignal<Double> leftStator = left.getStatorCurrent();
    private StatusSignal<Double> leftSupply = left.getSupplyCurrent();
    private StatusSignal<Double> leftTemp = left.getDeviceTemp();

    private StatusSignal<Double> rightStator = right.getStatorCurrent();
    private StatusSignal<Double> rightSupply = right.getSupplyCurrent();
    private StatusSignal<Double> rightTemp = right.getDeviceTemp();

    private REV10KPotentiometer encoder;
    private DigitalInput limitSwitch;

    public ArmIOPotentiometer() {
        encoder = new REV10KPotentiometer(ArmConstants.Potentiometer,
            ArmConstants.potentiometerMinVolt, ArmConstants.potentiometerMaxVolt,
            ArmConstants.potentiometerMaxAngle - ArmConstants.potentiometerMinAngle);
        limitSwitch = new DigitalInput(ArmConstants.LimitSwitch);

        configureMotors();
    }

    private void configureMotors() {
        left = new TalonFX(ArmConstants.LeftMotorID);
        left.setInverted(Subsystems.TalonFXDirectionCounterClockWise);

        right = new TalonFX(ArmConstants.RightMotorID);
        right.setControl(new StrictFollower(left.getDeviceID()));
        right.setInverted(Subsystems.TalonFXDirectionCounterClockWise);

        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
        BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, leftTemp,
            rightStator, rightSupply, rightTemp);
        registerMotors(left, right);
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
        inputs.position = encoder.getDegrees();
        inputs.potentiometerVoltage = encoder.getVoltage();
        inputs.limitSwitch = limitSwitch.get();

        if (inputs.limitSwitch) {
            resetPotentiometer();
        }
    }

    @Override
    public void setVoltage(double voltage) {
        if (!areMotorsEnabled()) return;
        appliedVoltage = voltage;
        left.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean coast) {
        left.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        right.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    @Override
    public void setPotentiometerBounds(double minVolt, double maxVolt) {
        encoder.setMinimumVolt(minVolt);
        encoder.setMaximumVolt(maxVolt);
    }

    private void resetPotentiometer() {
        double oldMaxVolt = encoder.getMaximumVolt();
        encoder.setMaximumVolt(encoder.getVoltage());
        double oldMinVolt = encoder.getMinimumVolt();
        encoder.setMaximumVolt((encoder.getMaximumVolt() - oldMaxVolt) + oldMinVolt);
    }
}
