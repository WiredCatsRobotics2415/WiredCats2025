package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import frc.utils.tuning.TuneableNumber;
import lombok.Getter;

public class TorqueMonitor {
    private TuneableNumber jumpThreshold;
    private TuneableNumber jumpMagnitude;
    private TuneableNumber tripTime;

    private Timer tripTimer;
    @Getter private boolean tripped;
    private boolean firstRun = true;
    private double lastCurrent;

    public TorqueMonitor(TuneableNumber jumpThreshold, TuneableNumber jumpMagnitude, TuneableNumber tripTime) {
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
        if (statorCurrent > jumpThreshold.get() && (statorCurrent - lastCurrent) > jumpMagnitude.get()) {
            tripTimer.start();
            return;
        }
        if (tripTimer.hasElapsed(tripTime.get())) {
            tripped = true;
            return;
        }
        if (tripTimer.isRunning() && statorCurrent < jumpThreshold.get()) {
            tripTimer.stop();
            tripTimer.reset();
        }
    }

    public void reset() {
        tripped = false;
    }
}
