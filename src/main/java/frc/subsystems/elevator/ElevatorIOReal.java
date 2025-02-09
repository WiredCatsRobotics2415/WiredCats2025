package frc.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Util;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

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
