package frc.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        public boolean motorConnected = true;
        public Current motorStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current motorSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Temperature motorTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);

        public double position = 0.0d;
        public double appliedVoltage = 0.0d;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setCoast(boolean coast) {}
}
