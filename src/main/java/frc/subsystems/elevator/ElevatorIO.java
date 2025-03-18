package frc.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean isConnectedLeft;
        public double temperatureLeft = 0.0d;
        public double supplyCurrentLeft = 0.0d;
        public double statorCurrentLeft = 0.0d;

        public boolean isConnectedRight;
        public double temperatureRight = 0.0d;
        public double supplyCurrentRight = 0.0d;
        public double statorCurrentRight = 0.0d;

        public double appliedVoltage = 0.0d;
        public double wirePotentiometer = 0.0d;
        public double inches = 0.0d;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setVoltage(double volts) {};

    public default void setCoast(boolean coast) {};
}
