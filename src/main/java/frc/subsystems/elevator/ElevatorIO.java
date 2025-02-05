package frc.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIO {

    public class ElevatorIOInputs {

        // I don't think I ever updated these variables...
        public double wirePotentiometerValue;

        public boolean isConnectedLeft;
        public double temperatureLeft;
        public double currentDrawLeft;

        public boolean isConnectedRight;
        public double temperatureRight;
        public double currentDrawRight;
    }

    public void setVoltage(double volts) {}
}
