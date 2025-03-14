package frc.utils.hardware;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.utils.tuning.TuneableBoolean;
import frc.utils.tuning.TuneableNumber;

public class Throughbore {
    private DutyCycleEncoder input;
    private TuneableNumber zero;
    private TuneableNumber one;
    private TuneableBoolean wrap;

    private double raw;

    public Throughbore(int channel, double valueToReturnZero, double valueToReturnOne, boolean doesWrap, String name) {
        input = new DutyCycleEncoder(channel);
        zero = new TuneableNumber(valueToReturnZero, name + "/valueToReturnZero");
        one = new TuneableNumber(valueToReturnOne, name + "/valueToReturnOne");
        wrap = new TuneableBoolean(doesWrap, name + "/doesWrap");
    }

    public double get() {
        raw = input.get();
        if (wrap.get() && raw < zero.get()) {
            return (raw - (zero.get() + 1)) / (one.get() - (zero.get() + 1));
        }
        return (raw - zero.get()) / (one.get() - zero.get());
    }

    /**
     * Raw throughbore value, only gets updated when get() is called
     */
    public double getRaw() { return raw; }
}
