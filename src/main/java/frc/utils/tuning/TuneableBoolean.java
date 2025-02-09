package frc.utils.tuning;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class TuneableBoolean {
    private static NetworkTable mainTable;

    private BooleanEntry thisEntry;
    private boolean previousValue;
    private String name;
    private ArrayList<Runnable> listeners;

    public TuneableBoolean(boolean defaultValue, String name) {
        this.previousValue = defaultValue;
        if (RuntimeConstants.TuningMode) {
            this.name = name;
            if (mainTable == null) mainTable = NetworkTableInstance.getDefault().getTable("/TuneableBooleans");
            thisEntry = mainTable.getBooleanTopic(name).getEntry(defaultValue);
            thisEntry.accept(defaultValue);
            listeners = new ArrayList<Runnable>();
        }
    }

    public boolean get() {
        if (RuntimeConstants.TuningMode) {
            boolean entry = thisEntry.get();
            if (entry != previousValue) {
                previousValue = entry;
                for (Runnable r : listeners)
                    r.run();
            }
            Logger.recordOutput("TuneableBooleans/" + name, entry);
            return entry;
        }
        return previousValue;
    }

    public void addListener(Runnable toCall) {
        listeners.add(toCall);
    }
}
