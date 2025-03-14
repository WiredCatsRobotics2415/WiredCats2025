package frc.utils.tuning;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class TuneableDistance extends TuneableNumber {
    private Distance lastMeasure;

    public TuneableDistance(double defaultInches, String key) {
        super(defaultInches, key);
        lastMeasure = Inches.of(defaultInches);
        this.addListener((newValue) -> lastMeasure = Inches.of(newValue));
    }

    public TuneableDistance(Distance initial, String key) {
        this(initial.in(Inches), key);
    }

    public double in(DistanceUnit unit) {
        return unit.convertFrom(get(), Inches);
    }

    public Distance distance() {
        return lastMeasure;
    }
}
