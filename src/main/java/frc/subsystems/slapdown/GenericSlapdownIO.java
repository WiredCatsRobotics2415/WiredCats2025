package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.littletonrobotics.junction.AutoLog;

public interface GenericSlapdownIO {
    @AutoLog
    abstract class GenericSlapdownIOInputs {
        public boolean pivotConnected = true;
        public Temperature pivotTemp = Celsius.of(0);
        public Current pivotStatorCurrent = Amps.of(0);

        public boolean intakeConnected = true;
        public Temperature intakeTemp = Celsius.of(0);
        public Current intakeStatorCurrent = Amps.of(0);

        public double throughborePosition = 0.0d;
        public double sensorValue = 0.0d;
    }

    public default void updateInputs(GenericSlapdownIOInputsAutoLogged inputs) {}

    /**
     * Configures hardware. If this slapdown does not have an IR sensor, make sensorAnalogPort -1;
     */
    public default void configureHardware(int pivotId, int intakeId, int throughborePort, int sensorAnalogPort) {};

    public default void configureSim(String targetGamePiece, Distance width, Distance lengthExtended, IntakeSide side,
        double rotorToGearRatio, Distance effectiveLength, Angle max, Angle min) {};

    public default void setPivotVoltage(double voltage) {};

    public default void setIntakePower(double power) {};
}
