package frc.utils.tuning;

import edu.wpi.first.math.jni.ArmFeedforwardJNI;
import frc.constants.RuntimeConstants;

public class TuneableArmFF {
    private double s, g, v, a;
    private TuneableNumber kS, kG, kV, kA;

    public TuneableArmFF(double kS, double kG, double kV, double kA, String controllerName) {
        this.s = kS;
        this.g = kG;
        this.v = kV;
        this.a = kA;
        if (RuntimeConstants.TuningMode) {
            this.kS = new TuneableNumber(kS, controllerName + "/kS");
            this.kG = new TuneableNumber(kG, controllerName + "/kG");
            this.kV = new TuneableNumber(kV, controllerName + "/kV");
            this.kA = new TuneableNumber(kA, controllerName + "/kA");

            this.kS.addListener(newNumber -> this.s = newNumber);
            this.kG.addListener(newNumber -> this.g = newNumber);
            this.kV.addListener(newNumber -> this.v = newNumber);
            this.kA.addListener(newNumber -> this.a = newNumber);
        }
    }

    public double calculate(double angleRads, double velocity) {
        return s * Math.signum(velocity) + g * Math.cos(angleRads) + v * velocity;
    }

    public double calculateWithVelocities(double currentAngleRads, double currentAngleVelocityRadsS,
        double nextAngleVelocityRadsS) {
        return ArmFeedforwardJNI.calculate(s, v, a, g, currentAngleRads, currentAngleVelocityRadsS,
            nextAngleVelocityRadsS, 0.02);
    }
}
