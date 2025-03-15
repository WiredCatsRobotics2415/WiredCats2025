package frc.utils.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneableNumber {
    private static ArrayList<TuneableNumber> all = new ArrayList<TuneableNumber>();
    private static int indexToUpdate = 0;

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

    public TuneableNumber(Distance defaultDistance, String key) {
        this(defaultDistance.in(Inches), key);
    }

    public TuneableNumber(Angle defaultAngle, String key) {
        this(defaultAngle.in(Degrees), key);
    }

    public double get() {
        return previousNumber;
    }

    /**
     * Assuming this number is a distance in inches, convert it to inches
     */
    public double meters() {
        return Units.inchesToMeters(previousNumber);
    }

    /**
     * Assuming this number is in inches
     */
    public Distance distance() {
        return Inches.of(previousNumber);
    }

    /**
     * Assuming this number is an angle in degrees, convert it to radians
     */
    public double radians() {
        return Units.degreesToRadians(previousNumber);
    }

    /**
     * Assuming this number is in degrees
     */
    public Angle angle() {
        return Degrees.of(previousNumber);
    }

    public void set(double newNumber) {
        if (RuntimeConstants.TuningMode) {
            previousNumber = newNumber;
            thisNetworkNumber.set(newNumber);
        }
    }

    /**
     * Consumer to call when value is changed from NT. Do NOT call get() on this tuneable from within the consumer, because the value will not have changed yet.
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
        all.get(indexToUpdate).updateFromNT();
        indexToUpdate += 1;
        if (indexToUpdate == all.size()) {
            indexToUpdate = 0;
        }
    }
}
