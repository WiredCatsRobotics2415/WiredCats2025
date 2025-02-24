package frc.subsystems.coralintake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class CoralIntakeIOSim implements CoralIntakeIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 1, 1),
        DCMotor.getFalcon500(1));

    public CoralIntakeIOSim() {}

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        inputs.motorConnected = true;
        inputs.motorStatorCurrent = Amps.of(motor.getCurrentDrawAmps());
        inputs.motorSupplyCurrent = Amps.of(0.0d);
        inputs.motorTemp = Celsius.of(0);
    }

    @Override
    public void setPower(double power) {
        motor.setInputVoltage(power * RoboRioSim.getVInVoltage());
    }
}
