package frc.constants;

public final class RuntimeConstants {
    /** What to do if in simulation */
    public static final SimMode simMode = SimMode.SIM;

    public static enum SimMode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean tuningMode = false;
    public static final boolean visualizationEnabledWhenReal = true;
}
