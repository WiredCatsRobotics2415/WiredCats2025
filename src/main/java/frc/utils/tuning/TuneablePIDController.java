package frc.utils.tuning;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.constants.RuntimeConstants;

public class TuneablePIDController extends PIDController {
    private TuneableNumber kP, kI, kD, tolerance;
    private double lastMeasure;
    private boolean frozen = false;

    public TuneablePIDController(double kP, double kI, double kD, String controllerName) {
        super(kP, kI, kD);
        if (RuntimeConstants.TuningMode) {
            this.kP = new TuneableNumber(kP, controllerName + "/kP");
            this.kI = new TuneableNumber(kI, controllerName + "/kI");
            this.kD = new TuneableNumber(kD, controllerName + "/kD");
            this.tolerance = new TuneableNumber(0, controllerName + "/tolerance");

            this.kP.addListener(newNumber -> this.setP(newNumber));
            this.kI.addListener(newNumber -> this.setI(newNumber));
            this.kD.addListener(newNumber -> this.setD(newNumber));
            this.tolerance.addListener(newNumber -> super.setTolerance(this.tolerance.get()));
        }
    }

    public void freeze() {
        frozen = true;
    }

    public void unfreeze() {
        frozen = false;
    }

    public void setConstraints(Constraints constraints) { frozen = constraints.maxVelocity == 0; }

    @Override
    public void setTolerance(double positionTolerance) {
        super.setTolerance(positionTolerance);
        if (RuntimeConstants.TuningMode) {
            this.tolerance.set(positionTolerance);
        }
    }

    @Override
    public double calculate(double measurement) {
        lastMeasure = measurement;
        if (frozen)
            return 0;
        else
            return super.calculate(measurement);
    }

    /**
     * @return Goal - measure
     */
    public double goalError() {
        return this.getSetpoint() - lastMeasure;
    }
}
