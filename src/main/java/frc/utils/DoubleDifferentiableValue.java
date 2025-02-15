package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class DoubleDifferentiableValue {
    @Getter private double value;
    @Getter private double velocity;
    @Getter private double acceleration;

    private boolean firstUpdate = false;
    private double lastTimestamp;
    private double lastVelocity;

    public DoubleDifferentiableValue() {}

    public void update(double update) {
        if (firstUpdate) {
            value = update;
            velocity = 0.0d;
            acceleration = 0.0d;
        } else {
            velocity = (update - value) / lastTimestamp;
            acceleration = (velocity - lastVelocity) / lastTimestamp;
        }
        lastTimestamp = Timer.getFPGATimestamp();
        lastVelocity = velocity;
        firstUpdate = false;
    }
}
