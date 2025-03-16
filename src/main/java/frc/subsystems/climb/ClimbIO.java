package frc.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    abstract class ClimbIOInputs {
        public boolean motorConnected = true;

        public double motorStatorCurrent = 0.0d;
        public double motorSupplyCurrent = 0.0d;
        public double motorTemp = 0.0d;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setPower(double power) {};
}
