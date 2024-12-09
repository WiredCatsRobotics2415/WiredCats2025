package frc.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.constants.Subsystems.IntakeConstants;
import frc.util.visualization.RobotVisualizer;

public class IntakeIOSim implements IntakeIO {
    private DCMotorSim motor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);
    private AnalogInputSim closeToFlywheelSensor = new AnalogInputSim(
        IntakeConstants.FlywheelIR);

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        // inputs.sensorTrigger = (closeToFlywheelSensor.getVoltage() / 5) * 4096.0d <
        // IntakeConstants.IRThreshold;
        inputs.sensorTrigger = RobotVisualizer.hasNote();

        inputs.motorStatorCurrent = motor.getCurrentDrawAmps();
        inputs.motorSupplyCurrent = 0.0d;
        inputs.motorTemp = 0.0d;
    }

    @Override
    public void on(double speed) {
        motor.setInputVoltage(speed * RoboRioSim.getVInVoltage());
    }

    @Override
    public void off() {
        motor.setInputVoltage(0);
    }
}
