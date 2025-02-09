package frc.utils.tuning;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class TuneableNumber {
    private static NetworkTable mainTable;

    private DoubleEntry thisEntry;
    private double previousNumber;
    private String name;
    private ArrayList<Runnable> listeners;

    public TuneableNumber(double defaultNumber, String name) {
        this.previousNumber = defaultNumber;
        if (RuntimeConstants.TuningMode) {
            this.name = name;
            if (mainTable == null) mainTable = NetworkTableInstance.getDefault().getTable("/TuneableNumbers");
            thisEntry = mainTable.getDoubleTopic(name).getEntry(defaultNumber);
            thisEntry.accept(defaultNumber);
            listeners = new ArrayList<Runnable>();
        }
    }

    public double get() {
        if (RuntimeConstants.TuningMode) {
            double entry = thisEntry.get();
            if (entry != previousNumber) {
                previousNumber = entry;
                for (Runnable r : listeners)
                    r.run();
            }
            Logger.recordOutput("TuneableNumbers/" + name, entry);
            return entry;
        }
        return previousNumber;
    }

    public void addListener(Runnable toCall) {
        listeners.add(toCall);
    }
}
