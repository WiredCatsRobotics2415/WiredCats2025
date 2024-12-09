package frc.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean leftConnected = true;
        public double leftVelocity = 0.0d;
        public double leftTemp = 0.0d;
        public double leftStatorCurrent = 0.0d;
        public double leftSupplyCurrent = 0.0d;
        public double leftAppliedVolts = 0.0d;

        public boolean rightConnected = true;
        public double rightVelocity = 0.0d;
        public double rightTemp = 0.0d;
        public double rightStatorCurrent = 0.0d;
        public double rightSupplyCurrent = 0.0d;
        public double rightAppliedVolts = 0.0d;
    }

    public default void updateInputs(FlywheelIOInputsAutoLogged inputs) {}

    public default void setRPM(double leftRPM, double rightRPM) {}
}
