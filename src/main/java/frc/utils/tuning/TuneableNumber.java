package frc.utils.tuning;

import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneableNumber {
    private static ArrayList<TuneableNumber> all = new ArrayList<TuneableNumber>();

    private LoggedNetworkNumber thisNetworkNumber;
    private double previousNumber;
    private ArrayList<Consumer<Double>> listeners;
    private String key;

    public TuneableNumber(double defaultNumber, String key) {
        this.previousNumber = defaultNumber;
        this.key = key;
        if (RuntimeConstants.TuningMode) {
            thisNetworkNumber = new LoggedNetworkNumber("/Tuning/" + key);
            thisNetworkNumber.setDefault(defaultNumber);
            listeners = new ArrayList<Consumer<Double>>();
            all.add(this);
        }
    }

    public double get() {
        return previousNumber;
    }

    public void set(double newNumber) {
        if (RuntimeConstants.TuningMode) {
            previousNumber = newNumber;
            thisNetworkNumber.set(newNumber);
        }
    }

    /**
     * Consumer to call when value is changed from NT. Do NOT call get() on this tuneable, because the value will not have changed yet.
     *
     * @param toCall
     */
    public void addListener(Consumer<Double> toCall) {
        listeners.add(toCall);
    }

    private void updateFromNT() {
        double entry = thisNetworkNumber.get();
        if (entry != previousNumber) {
            previousNumber = entry;
            for (Consumer<Double> r : listeners)
                r.accept(entry);
        }
    }

    public static void periodic() {
        for (TuneableNumber number : all) {
            number.updateFromNT();
        }
    }
}
