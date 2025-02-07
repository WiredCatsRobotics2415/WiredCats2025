package frc.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        public boolean leftConnected = true;
        public Current leftStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current leftSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Temperature leftTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);

        public boolean rightConnected = true;
        public Current rightStatorCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Current rightSupplyCurrent = Current.ofBaseUnits(0.0, Units.Amps);
        public Temperature rightTemp = Temperature.ofBaseUnits(0.0, Units.Celsius);

        public double position = 0.0d;
        public double appliedVoltage = 0.0d;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setCoast(boolean coast) {}
}