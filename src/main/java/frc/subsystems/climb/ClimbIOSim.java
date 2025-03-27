package frc.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 25),
        DCMotor.getKrakenX60(1));
    double appliedVolts;

    public ClimbIOSim() {

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.motorSupplyCurrent = motor.getCurrentDrawAmps();
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        motor.setInputVoltage(volts);
    }
}
