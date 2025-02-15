package frc.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public boolean isConnected;
        public double temperature;
        public double currentDraw;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {};

    public default void setVoltage(double volts) {};
}
