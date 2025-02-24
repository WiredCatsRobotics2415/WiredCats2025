package frc.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Subsystems.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor = new TalonFX(ElevatorConstants.LeftMotorPort);
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.RightMotorPort);
    private double appliedVoltage;

    private StatusSignal<Current> leftStator = leftMotor.getStatorCurrent();
    private StatusSignal<Current> leftSupply = leftMotor.getSupplyCurrent();
    private StatusSignal<Temperature> leftTemp = leftMotor.getDeviceTemp();

    private StatusSignal<Current> rightStator = rightMotor.getStatorCurrent();
    private StatusSignal<Current> rightSupply = rightMotor.getSupplyCurrent();
    private StatusSignal<Temperature> rightTemp = rightMotor.getDeviceTemp();

    private AnalogInput wirePotentiometer;

    public ElevatorIOReal() {
        wirePotentiometer = new AnalogInput(ElevatorConstants.AnalogPotentiometerPort);
        // configureMotors();
    }

    private void configureMotors() {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        leftMotor = new TalonFX(ElevatorConstants.LeftMotorPort);
        leftMotor.getConfigurator().apply(feedbackConfigs);

        rightMotor = new TalonFX(ElevatorConstants.RightMotorPort);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.wirePotentiometerValue = wirePotentiometer.getVoltage();
    }

    public void setVoltage(double voltOut) {
        // leftMotor.setVoltage(voltOut);
    }
}
