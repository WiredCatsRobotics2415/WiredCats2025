package frc.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIO {
    

    public class ElevatorIOInputs {
        public double wirePotentiometerValue; 
        
        
        public boolean isConnectedLeft; 
        public double temperatureLeft; 
        public double currentDrawLeft; 

        public boolean isConnectedRight; 
        public double temperatureRight; 
        public double currentDrawRight; 
    }

    public void setVoltage(double volts) {

    }
    // function from elevator code (to-be altered here): 
    /* 
    public void setVoltage(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity); 
        double voltOut = output + feedforward; 
        // set motors based on voltOut
        if (!isCoasting) {
            // leftMotor.setVoltage(voltOut); 
        }
        SmartDashboard.putNumber("Elevator Volt out", voltOut); 
    }
        */
}
