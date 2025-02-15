package frc.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        public boolean motorConnected = true;
        public Current motorStatorCurrent = Amps.of(0.0d);
        public Current motorSupplyCurrent = Amps.of(0.0d);
        public Temperature motorTemp = Celsius.of(0.0d);

        public double throughborePosition = 0.0d;
        public Voltage appliedVoltage = Volts.of(0.0d);
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setCoast(boolean coast) {}
}
