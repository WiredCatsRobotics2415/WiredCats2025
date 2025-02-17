package frc.utils.math;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class DoubleDifferentiableValue {
    @Getter private double value;
    @Getter private double firstDerivative;
    @Getter private double secondDerivative;

    private boolean firstUpdate = false;
    private double lastTimestamp;
    private double lastVelocity;

    public DoubleDifferentiableValue() {}

    public void update(double update) {
        if (firstUpdate) {
            value = update;
            firstDerivative = 0.0d;
            secondDerivative = 0.0d;
        } else {
            firstDerivative = (update - value) / lastTimestamp;
            secondDerivative = (firstDerivative - lastVelocity) / lastTimestamp;
        }
        lastTimestamp = Timer.getFPGATimestamp();
        lastVelocity = firstDerivative;
        firstUpdate = false;
    }
}
