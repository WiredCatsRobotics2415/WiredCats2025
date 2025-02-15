package frc.subsystems.endeffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    abstract class EndEffectorIOInputs {
        public boolean motorConnected = true;
        public Temperature motorTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);
        public Current motorStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current motorSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);

        public int sensorValue = 0;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void setPower(double power) {};
}
