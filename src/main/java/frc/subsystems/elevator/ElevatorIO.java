package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean isConnectedLeft;
        public Temperature temperatureLeft = Celsius.of(0.0d);
        public Current supplyCurrentLeft = Amps.of(0.0d);
        public Current statorCurrentLeft = Amps.of(0.0d);

        public boolean isConnectedRight;
        public Temperature temperatureRight = Celsius.of(0.0d);
        public Current supplyCurrentRight = Amps.of(0.0d);
        public Current statorCurrentRight = Amps.of(0.0d);

        public Voltage appliedVoltage = Volts.of(0.0d);
        public double wirePotentiometer = 0.0d;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setVoltage(double volts) {};

    public default void setCoast(boolean coast) {};
}
