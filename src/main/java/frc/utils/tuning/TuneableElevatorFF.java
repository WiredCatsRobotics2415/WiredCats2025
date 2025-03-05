package frc.utils.tuning;

import frc.constants.RuntimeConstants;

public class TuneableElevatorFF {
    private double s, v, g, a;
    private TuneableNumber kS, kV, kG, kA;

    public TuneableElevatorFF(double kS, double kV, double kG, double kA, String controllerName) {
        this.s = kS;
        this.v = kV;
        this.g = kG;
        this.a = kA;

        if (RuntimeConstants.TuningMode) {
            this.kS = new TuneableNumber(kS, controllerName + "/kS");
            this.kV = new TuneableNumber(kV, controllerName + "/kV");
            this.kG = new TuneableNumber(kG, controllerName + "/kG");
            this.kA = new TuneableNumber(kA, controllerName + "/kA");

            this.kS.addListener(newNumber -> this.s = newNumber);
            this.kV.addListener(newNumber -> this.v = newNumber);
            this.kG.addListener(newNumber -> this.g = newNumber);
            this.kA.addListener(newNumber -> this.a = newNumber);
        }
    }

    /**
     * Identical to ElevatorFeedfoward.calculate
     */
    public double calculate(double velocity) {
        return s * Math.signum(velocity) + g + v * velocity;
    }

    /**
     * Identical to ElevatorFeedfoward.calculateWithVelocities
     */
    public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
        if (a == 0.0) {
            return s * Math.signum(nextVelocity) + g + v * nextVelocity;
        } else {
            double A = -v / a;
            double B = 1.0 / a;
            double A_d = Math.exp(A * 0.02);
            double B_d = 1.0 / A * (A_d - 1.0) * B;
            return g + s * Math.signum(currentVelocity) + 1.0 / B_d * (nextVelocity - A_d * currentVelocity);
        }
    }
}
