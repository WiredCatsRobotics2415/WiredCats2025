package frc.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        public boolean motorConnected = true;
        public double motorStatorCurrent = 0.0d;
        public double motorSupplyCurrent = 0.0d;
        public double motorTemp = 0.0d;

        public double throughborePosition = 0.0d;
        public double appliedVoltage = 0.0d;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setCoast(boolean coast) {}
}
