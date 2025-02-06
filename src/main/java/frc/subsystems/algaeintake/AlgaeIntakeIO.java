package frc.subsystems.algaeintake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
    @AutoLog
    abstract class AlgaeIntakeIOInputs {
        public boolean limitSwitch = false;

        public boolean motorConnected = true;
        public Temperature motorTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);
        public Current motorStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current motorSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
    }

    public default void updateInputs(AlgaeIntakeIOInputsAutoLogged inputs) {}

    public default void setPower(double speed) {}

    public default void off() {}
}
