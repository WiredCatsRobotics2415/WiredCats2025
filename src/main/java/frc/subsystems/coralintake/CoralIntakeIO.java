package frc.subsystems.coralintake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
    @AutoLog
    abstract class CoralIntakeIOInputs {
        public boolean pivotConnected = true;
        public double pivotTemp = 0.0d;
        public double pivotStatorCurrent = 0.0d;
        public double appliedVoltage = 0.0d;

        public boolean intakeConnected = true;
        public double intakeTemp = 0.0d;
        public double intakeStatorCurrent = 0.0d;
        public double appliedPower = 0.0d;

        public double throughborePosition = 0.0d;
    }

    public default void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {}

    public default void setPivotVoltage(double voltage) {};

    public default void setIntakePower(double power) {};
}
