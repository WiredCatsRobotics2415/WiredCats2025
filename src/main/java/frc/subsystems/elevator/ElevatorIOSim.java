package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Util;

public class ElevatorIOSim implements ElevatorIO {
    private double appliedVoltage;

    private final ElevatorSim simElevator = new ElevatorSim(ElevatorConstants.kV, ElevatorConstants.kA,
        DCMotor.getFalcon500(2), ElevatorConstants.MinHeightInches.in(Meters),
        ElevatorConstants.MaxHeightInches.in(Meters), true, ElevatorConstants.MinHeightInches.in(Meters));

    public ElevatorIOSim() {
        simElevator.setState(0, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simElevator.update(0.02);

        inputs.appliedVoltage = Volts.of(appliedVoltage);

        Current draw = Amps.of(simElevator.getCurrentDrawAmps());
        inputs.isConnectedLeft = true;
        inputs.statorCurrentLeft = draw;

        inputs.isConnectedRight = true;
        inputs.statorCurrentRight = draw;

        inputs.wirePotentiometer = Util.linearMap(simElevator.getPositionMeters(),
            ElevatorConstants.MinHeightInches.in(Meters), ElevatorConstants.MaxHeightInches.in(Meters),
            ElevatorConstants.PotentiometerMinVolt.in(Volts), ElevatorConstants.PotentiometerMaxVolt.in(Volts));
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
        simElevator.setInputVoltage(volts);
    }
}
