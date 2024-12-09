package frc.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    abstract class ClawIOInputs {
        public double position = 0.0d;

        public double motorStatorCurrent = 0.0d;
        public double motorTemp = 0.0d;
        public double appliedVoltage = 0.0d;
    }

    public default void updateInputs(ClawIOInputsAutoLogged inputs) {}

    public default void setPosition(double position) {}

    public default void setEncoderPosition(double position) {}
}
