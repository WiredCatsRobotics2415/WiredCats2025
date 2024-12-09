package frc.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        public boolean leftConnected = true;
        public double leftStatorCurrent = 0.0d;
        public double leftSupplyCurrent = 0.0d;
        public double leftTemp = 0.0d;

        public boolean rightConnected = true;
        public double rightStatorCurrent = 0.0d;
        public double rightSupplyCurrent = 0.0d;
        public double rightTemp = 0.0d;

        public double position = 0.0d;
        public boolean limitSwitch = false;
        public double appliedVoltage = 0.0d;
        public double potentiometerVoltage = 0.0d;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setCoast(boolean coast) {}

    public default void setPotentiometerBounds(double minVolt, double maxVolt) {}
}
