package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class TorqueMonitor {
    private double jumpThreshold;
    private double jumpMagnitude;
    private double tripTime;

    private Timer tripTimer;
    @Getter private boolean tripped;
    private boolean firstRun = true;
    private double lastCurrent;

    public TorqueMonitor(double jumpThreshold, double jumpMagnitude, double tripTime) {
        this.jumpThreshold = jumpThreshold;
        this.jumpMagnitude = jumpMagnitude;
        this.tripTime = tripTime;
        this.tripTimer = new Timer();
        this.tripTimer.reset();
    }

    public void update(double statorCurrent) {
        if (firstRun) {
            lastCurrent = statorCurrent;
            firstRun = false;
            return;
        }
        if (tripped == true) {
            firstRun = true;
            return;
        }
        if (statorCurrent > jumpThreshold && (statorCurrent - lastCurrent) > jumpMagnitude) {
            tripTimer.start();
            return;
        }
        if (tripTimer.hasElapsed(tripTime)) {
            tripped = true;
            return;
        }
        if (tripTimer.isRunning() && statorCurrent < jumpThreshold) {
            tripTimer.stop();
            tripTimer.reset();
        }
    }

    public void reset() {
        tripped = false;
    }
}
