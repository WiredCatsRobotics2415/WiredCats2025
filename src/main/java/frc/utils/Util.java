package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.constants.RuntimeConstants;
import frc.constants.RuntimeConstants.Mode;
import frc.robot.Robot;
import java.lang.reflect.InvocationTargetException;

public class Util {
    /**
     * Select the correct IO implementation based on the runtime mode.
     *
     * @param real   The IO if the robot is on real hardware
     * @param sim    The physics simulation IO
     * @param replay The replay IO
     * @return The instantiated IO (MUST be cast to correct IO class)
     */
    public static Object getIOImplementation(Class<?> real, Class<?> sim, Class<?> replay) {
        Class<?> attemptedInstantationType = real;
        try {
            if (Robot.isSimulation()) {
                if (RuntimeConstants.simMode == Mode.REPLAY) {
                    attemptedInstantationType = replay;
                } else {
                    attemptedInstantationType = sim;
                }
            } else {
                attemptedInstantationType = real;
            }
            return attemptedInstantationType.getDeclaredConstructors()[0].newInstance();
        } catch (InstantiationException | SecurityException | IllegalAccessException | InvocationTargetException e) {
            e.printStackTrace();
            System.out.println("COULD NOT INSTANTIATE IO IMPLEMENTATION FOR CLASS "
                + attemptedInstantationType.getName() + "; SEE ERROR MESSAGE ABOVE (NOT THE CASTEXCEPTION)");
            new Alert("COULD NOT INSTANTIATE IO IMPLEMENTATION FOR CLASS " + attemptedInstantationType.getName(),
                AlertType.kError).set(true);

        }
        return new Object();
    }
}
