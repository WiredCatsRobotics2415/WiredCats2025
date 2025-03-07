package frc.utils.tuning;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;

public class TuneableTime extends TuneableNumber {
    public TuneableTime(double defaultSeconds, String key) {
        super(defaultSeconds, key);
    }

    public TuneableTime(Time initial, String key) {
        super(initial.in(Seconds), key);
    }

    public double in(TimeUnit unit) {
        return unit.convertFrom(get(), Seconds);
    }

    public Time time() {
        return Seconds.of(get());
    }
}
