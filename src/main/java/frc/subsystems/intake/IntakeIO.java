package frc.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    abstract class IntakeIOInputs {
        public boolean sensorTrigger = false;

        public boolean motorConnected = true;
        public double motorTemp = 0.0d;
        public double motorStatorCurrent = 0.0d;
        public double motorSupplyCurrent = 0.0d;
    }

    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

    public default void on(double speed) {}

    public default void off() {}
}
