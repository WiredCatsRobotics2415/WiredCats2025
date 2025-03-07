package frc.utils.tuning;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class TuneableDistance extends TuneableNumber {
    public TuneableDistance(double defaultInches, String key) {
        super(defaultInches, key);
    }

    public double in(DistanceUnit unit) {
        return unit.convertFrom(get(), Inches);
    }

    public Distance asMeasure() {
        return Inches.of(get());
    }
}
