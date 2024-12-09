package frc.subsystems.claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.constants.Subsystems.ClawConstants;

public class ClawIOSim implements ClawIO {
    private DCMotorSim motor = new DCMotorSim(DCMotor.getNeo550(1), ClawConstants.GearRatio,
        0.01);
    private PIDController pidController = new PIDController(ClawConstants.Kp, 0,
        ClawConstants.Kd);
    private double appliedVoltage = 0.0d;

    public ClawIOSim() {}

    @Override
    public void updateInputs(ClawIOInputsAutoLogged inputs) {
        inputs.position = motor.getAngularPositionRotations();

        inputs.motorStatorCurrent = motor.getCurrentDrawAmps();
        inputs.motorTemp = 0.0d;
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setPosition(double position) {
        pidController.setSetpoint(position);
        appliedVoltage = pidController.calculate(motor.getAngularPositionRotations());
        motor.setInputVoltage(appliedVoltage);
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.setState(position * 2 * Math.PI, 0);
    }
}
