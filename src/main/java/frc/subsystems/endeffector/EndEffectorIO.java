package frc.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    abstract class EndEffectorIOInputs {
        public boolean motorConnected = true;
        public double motorTemp = 0.0d;
        public double motorStatorCurrent = 0.0d;
        public double motorSupplyCurrent = 0.0d;

        public int sensorValue = 0;
        public double appliedPower;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void setPower(double power) {};
}
