package frc.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double wirePotentiometerValue;

        public boolean isConnectedLeft;
        public double temperatureLeft;
        public double currentDrawLeft;

        public boolean isConnectedRight;
        public double temperatureRight;
        public double currentDrawRight;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setVoltage(double volts) {};
}
