package frc.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class RuntimeConstants {
    public static final Mode SimMode = Mode.SIM;
    public static final Mode CurrentMode = RobotBase.isReal() ? Mode.REAL : SimMode;
    public static final boolean TuningMode = false;
    public static final boolean VisualizationEnabled = true;

    public static final String HootFileName = "";

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
