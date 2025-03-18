package frc.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.math.Algebra;

public class ElevatorIOSim implements ElevatorIO {
    private double appliedVoltage;

    private final ElevatorSim simElevator = new ElevatorSim(0.01, 0.02, DCMotor.getFalcon500(2),
        ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight, true, ElevatorConstants.MinHeight);

    public ElevatorIOSim() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simElevator.update(0.02);

        inputs.appliedVoltage = appliedVoltage;

        double draw = simElevator.getCurrentDrawAmps() / 2;
        inputs.isConnectedLeft = true;
        inputs.statorCurrentLeft = draw;

        inputs.isConnectedRight = true;
        inputs.statorCurrentRight = draw;

        inputs.wirePotentiometer = Algebra.linearMap(simElevator.getPositionMeters(), ElevatorConstants.MinHeight,
            ElevatorConstants.MaxHeight, ElevatorConstants.PotentiometerMinVolt,
            ElevatorConstants.PotentiometerMaxVolt);
        inputs.inches = Algebra.linearMap(simElevator.getPositionMeters(), 0,
            Units.inchesToMeters(ElevatorConstants.MaxHeight), ElevatorConstants.MinHeight,
            ElevatorConstants.MaxHeight);
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
