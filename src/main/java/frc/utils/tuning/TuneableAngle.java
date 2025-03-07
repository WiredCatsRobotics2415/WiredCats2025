package frc.utils.tuning;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class TuneableAngle extends TuneableNumber {
    public TuneableAngle(double defaultDegrees, String key) {
        super(defaultDegrees, key);
    }

    public TuneableAngle(Angle initial, String key) {
        super(initial.in(Degrees), key);
    }

    public double in(AngleUnit unit) {
        return unit.convertFrom(get(), Degrees);
    }

    public Angle angle() {
        return Degrees.of(get());
    }
}
