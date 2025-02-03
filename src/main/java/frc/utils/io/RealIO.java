package frc.utils.io;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.constants.RuntimeConstants;

/**
 * Copied from 2024 code
 * All IOs that work with real hardware should extend this class
 *
 * <p>
 * Provides default method to check if motors are enabled (ie. this subsystem has not been disabled) so actuation methods can have a way to disable themselves
 *
 * <p>
 */
public class RealIO {
    private boolean motorsEnabled;
    private MotorController[] registeredMotors;

    public void setActuatorsEnabled(boolean enable) {
        if (enable) {
            motorsEnabled = true;
        } else {
            for (MotorController m : registeredMotors) {
                m.stopMotor();
            }
            motorsEnabled = false;
        }
    }

    public boolean areMotorsEnabled() {
        return motorsEnabled;
    }

    /**
     * Register all motors in the io. Calls optimizeBusUtilization on TalonFXs only if tuningMode is false
     *
     * @param motors motors in the io with their names that are displayed on
     */
    @SuppressWarnings("unused")
    public void registerMotors(MotorController... motors) {
        registeredMotors = motors;
        for (MotorController m : motors) {
            if (m instanceof TalonFX && !RuntimeConstants.tuningMode) {
                ((TalonFX) m).optimizeBusUtilization();
            }
        }
    }
}
