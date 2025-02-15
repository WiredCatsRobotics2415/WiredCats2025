package frc.subsystems.coralintake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
    @AutoLog
    abstract class CoralIntakeIOInputs {
        public boolean motorConnected = true;
        public Temperature motorTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);
        public Current motorStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current motorSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
    }

    public default void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {}

    public default void setPower(double power) {}
}
