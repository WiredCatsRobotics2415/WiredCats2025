package frc.subsystems.coralintake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.constants.Subsystems.CoralIntakeConstants;

public class CoralIntakeIOSim implements CoralIntakeIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 1, 1),
        DCMotor.getFalcon500(1));
    private DIOSim sensorInput = new DIOSim(new DigitalInput(0));

    public CoralIntakeIOSim() {}

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        // needs to get value from visualizer once it is implemented
        inputs.sensorValue = sensorInput.getValue() ? CoralIntakeConstants.IRThreshold : 0;

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = Current.ofBaseUnits(motor.getCurrentDrawAmps(), Units.Amps);
        inputs.motorSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        inputs.motorTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);
    }

    @Override
    public void setPower(double speed) {
        motor.setInputVoltage(speed * RoboRioSim.getVInVoltage());
    }
}
