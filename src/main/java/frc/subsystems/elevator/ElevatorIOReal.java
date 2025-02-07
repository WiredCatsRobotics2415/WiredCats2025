package frc.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.constants.Subsystems.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    public ElevatorIOReal() {
        // configureMotors();
    }

    public void configureMotors() {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        leftMotor = new TalonFX(ElevatorConstants.LeftMotorPort);
        leftMotor.getConfigurator().apply(feedbackConfigs);

        rightMotor = new TalonFX(ElevatorConstants.RightMotorPort);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setVoltage(double voltOut) {
        // leftMotor.setVoltage(voltOut);
    }
}
