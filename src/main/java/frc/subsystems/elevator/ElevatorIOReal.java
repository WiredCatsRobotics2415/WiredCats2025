package frc.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.constants.Subsystems.ElevatorConstants;

public class ElevatorIOReal {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    public void configureMotors() {
        // two talon FXs, neither inverted
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_PORT);
        leftMotor.getConfigurator().apply(feedbackConfigs);

        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_PORT);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void brake() {
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setVoltage(double voltOut) {
        leftMotor.setVoltage(voltOut);
    }
}
