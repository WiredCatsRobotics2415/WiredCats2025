package frc.utils.tuning;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class TuneableAngle extends TuneableNumber {
    private Angle lastMeasure;

    public TuneableAngle(double defaultDegrees, String key) {
        super(defaultDegrees, key);
        lastMeasure = Degrees.of(defaultDegrees);
        this.addListener((newValue) -> lastMeasure = Degrees.of(newValue));
    }

    public TuneableAngle(Angle initial, String key) {
        this(initial.in(Degrees), key);
    }

    public double in(AngleUnit unit) {
        return unit.convertFrom(get(), Degrees);
    }

    public Angle angle() {
        return lastMeasure;
    }
}
