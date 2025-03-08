package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.math.Algebra;

public class ElevatorIOSim implements ElevatorIO {
    private double appliedVoltage;

    private final ElevatorSim simElevator = new ElevatorSim(ElevatorConstants.kV, ElevatorConstants.kA,
        DCMotor.getFalcon500(2), ElevatorConstants.MinHeight.in(Meters), ElevatorConstants.MaxHeight.in(Meters), true,
        ElevatorConstants.MinHeight.in(Meters));

    public ElevatorIOSim() {

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

        inputs.wirePotentiometer = Algebra.linearMap(simElevator.getPositionMeters(),
            ElevatorConstants.MinHeight.in(Meters), ElevatorConstants.MaxHeight.in(Meters),
            ElevatorConstants.PotentiometerMinVolt.in(Volts), ElevatorConstants.PotentiometerMaxVolt.in(Volts));
    }

    @Override
    public void setVoltage(double volts) {
        if (RobotState.isDisabled()) {
            simElevator.setInputVoltage(0);
            appliedVoltage = 0;
            return;
        }
        appliedVoltage = volts;
        simElevator.setInputVoltage(volts);
    }
}
