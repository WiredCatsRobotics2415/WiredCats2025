package frc.subsystems.slapdown;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.littletonrobotics.junction.AutoLog;

public interface GenericSlapdownIO {
    @AutoLog
    abstract class GenericSlapdownIOInputs {
        public boolean pivotConnected = true;
        public double pivotTemp = 0.0d;
        public double pivotStatorCurrent = 0.0d;
        public double appliedVoltage = 0.0d;

        public boolean intakeConnected = true;
        public double intakeTemp = 0.0d;
        public double intakeStatorCurrent = 0.0d;

        public double throughborePosition = 0.0d;
        public double sensorValue = 0.0d;
    }

    public default void updateInputs(GenericSlapdownIOInputsAutoLogged inputs) {}

    /**
     * Configures hardware. If this slapdown does not have an IR sensor, make sensorAnalogPort -1;
     */
    public default void configureHardware(int pivotId, int intakeId, int throughborePort, double tBorMin,
        double tBorMax, boolean tBorWrap, int sensorAnalogPort) {};

    public default void configureSim(String targetGamePiece, Distance width, Distance lengthExtended, IntakeSide side,
        double rotorToGearRatio, Distance effectiveLength, Angle max, Angle min, double throughboreMin,
        double throughboreMax, Mass mass) {};

    public default void setPivotVoltage(double voltage) {};

    public default void setIntakePower(double power) {};
}
