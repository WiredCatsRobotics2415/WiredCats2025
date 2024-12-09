package frc.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.constants.Subsystems.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
    private LinearSystem<N1, N1, N1> leftLinearSystem = LinearSystemId
        .identifyVelocitySystem(FlywheelConstants.LeftMotorPID.kV, 1e-4); // effectively 0
    private LinearSystem<N1, N1, N1> rightLinearSystem = LinearSystemId
        .identifyVelocitySystem(FlywheelConstants.RightMotorPID.kV, 1e-4); // effectively 0
    private FlywheelSim left = new FlywheelSim(leftLinearSystem, DCMotor.getFalcon500(1),
        FlywheelConstants.GearRatio);
    private FlywheelSim right = new FlywheelSim(rightLinearSystem, DCMotor.getFalcon500(1),
        FlywheelConstants.GearRatio);

    private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0,
        FlywheelConstants.LeftMotorPID.kV);
    private SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0,
        FlywheelConstants.RightMotorPID.kV);
    private PIDController leftPID = new PIDController(FlywheelConstants.LeftMotorPID.kP, 0,
        0);
    private PIDController rightPID = new PIDController(FlywheelConstants.RightMotorPID.kD,
        0, 0);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    @Override
    public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
        left.update(0.02);
        right.update(0.02);

        inputs.leftVelocity = left.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftStatorCurrent = left.getCurrentDrawAmps();

        inputs.rightVelocity = right.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightStatorCurrent = right.getCurrentDrawAmps();
    }

    @Override
    public void setRPM(double leftRPM, double rightRPM) {
        leftAppliedVolts = leftFF.calculate(FlywheelConstants.rpmToRPS(leftRPM)) +
            leftPID.calculate(FlywheelConstants.rpmToRPS(leftRPM));
        rightAppliedVolts = rightFF.calculate(FlywheelConstants.rpmToRPS(rightRPM)) +
            rightPID.calculate(FlywheelConstants.rpmToRPS(rightRPM));
        left.setInputVoltage(leftAppliedVolts);
        right.setInputVoltage(rightAppliedVolts);
    }
}
