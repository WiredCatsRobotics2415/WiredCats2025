package frc.subsystems.endeffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.constants.Subsystems.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
    private SparkMax motor;

    public EndEffectorIOReal() {

    }

    public void configureMotor() {
        motor = new SparkMax(EndEffectorConstants.MotorPort, MotorType.kBrushless);
    }

    public void setVoltage(double voltOut) {
        motor.setVoltage(voltOut);
    }

}
