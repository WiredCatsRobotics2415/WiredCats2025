package frc.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.constants.Subsystems;
import frc.constants.Subsystems.FlywheelConstants;
import frc.util.io.RealIO;

public class FlywheelIOReal extends RealIO implements FlywheelIO {
    private TalonFX left = new TalonFX(FlywheelConstants.LeftMotorID);
    private TalonFX right = new TalonFX(FlywheelConstants.RightMotorID);
    private VelocityVoltage voltageVelocity;

    private StatusSignal<Double> leftRotorVelocity = left.getRotorVelocity();
    private StatusSignal<Double> leftTemp = left.getDeviceTemp();
    private StatusSignal<Double> leftStatorCurrent = left.getStatorCurrent();
    private StatusSignal<Double> leftSupplyCurrent = left.getSupplyCurrent();
    private StatusSignal<Double> leftAppliedVolts = left.getMotorVoltage();

    private StatusSignal<Double> rightRotorVelocity = right.getRotorVelocity();
    private StatusSignal<Double> rightTemp = right.getDeviceTemp();
    private StatusSignal<Double> rightStatorCurrent = right.getStatorCurrent();
    private StatusSignal<Double> rightSupplyCurrent = right.getSupplyCurrent();
    private StatusSignal<Double> rightAppliedVolts = right.getMotorVoltage();

    public FlywheelIOReal() {
        voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        configMotors();
    }

    private void configMotors() {
        TalonFXConfigurator rightCfg = right.getConfigurator();
        rightCfg.apply(FlywheelConstants.RightMotorPID);
        rightCfg.apply(FlywheelConstants.CoastConfig);
        rightCfg.apply(FlywheelConstants.CurrentLimits);
        right.setInverted(Subsystems.TalonFXDirectionCounterClockWise);

        TalonFXConfigurator leftCfg = left.getConfigurator();
        leftCfg.apply(FlywheelConstants.LeftMotorPID);
        leftCfg.apply(FlywheelConstants.CoastConfig);
        leftCfg.apply(FlywheelConstants.CurrentLimits);
        left.setInverted(Subsystems.TalonFXDirectionClockWise);

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftRotorVelocity, leftTemp,
            leftStatorCurrent, leftSupplyCurrent, leftAppliedVolts, rightRotorVelocity,
            rightTemp, rightStatorCurrent, rightSupplyCurrent, rightAppliedVolts);
        registerMotors(left, right);
    }

    @Override
    public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(leftRotorVelocity, leftTemp, leftStatorCurrent,
            leftSupplyCurrent, leftAppliedVolts, rightRotorVelocity, rightTemp,
            rightStatorCurrent, rightSupplyCurrent, rightAppliedVolts);

        inputs.leftConnected = left.isAlive();
        inputs.leftVelocity = FlywheelConstants.rpsToRPM(leftRotorVelocity.getValue());
        inputs.leftTemp = leftTemp.getValue();
        inputs.leftStatorCurrent = leftStatorCurrent.getValue();
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
        inputs.leftAppliedVolts = leftAppliedVolts.getValue();

        inputs.rightConnected = right.isAlive();
        inputs.rightVelocity = FlywheelConstants.rpsToRPM(rightRotorVelocity.getValue());
        inputs.rightTemp = rightTemp.getValue();
        inputs.rightStatorCurrent = rightStatorCurrent.getValue();
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
        inputs.rightAppliedVolts = rightAppliedVolts.getValue();
    }

    @Override
    public void setRPM(double leftRPM, double rightRPM) {
        if (!areMotorsEnabled()) return;
        left.setControl(voltageVelocity.withVelocity(FlywheelConstants.rpmToRPS(leftRPM)));
        right
            .setControl(voltageVelocity.withVelocity(FlywheelConstants.rpmToRPS(rightRPM)));
    }
}
