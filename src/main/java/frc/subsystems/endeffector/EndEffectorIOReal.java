package frc.subsystems.endeffector;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.constants.Subsystems.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
    private SparkMax motor;

    public EndEffectorIOReal() {

    }

    public void configureMotor() {
        motor = new SparkMax(EndEffectorConstants.MotorPort, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
        // weird error w/ spark, commenting so that I can commit. 
        // spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltOut) {
        motor.setVoltage(voltOut);
    }

}
