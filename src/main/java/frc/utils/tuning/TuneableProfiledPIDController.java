package frc.utils.tuning;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.constants.RuntimeConstants;
import lombok.Getter;

public class TuneableProfiledPIDController extends ProfiledPIDController {
    private TuneableNumber kP, kI, kD, veloMax, accelMax, tolerance;
    private double lastMeasure;
    @Getter private double lastCalculation = Double.MAX_VALUE;

    public TuneableProfiledPIDController(double kP, double kI, double kD, Constraints baseConstraints,
        String controllerName) {
        super(kP, kI, kD, baseConstraints);
        if (RuntimeConstants.TuningMode) {
            this.kP = new TuneableNumber(kP, controllerName + "/kP");
            this.kI = new TuneableNumber(kI, controllerName + "/kI");
            this.kD = new TuneableNumber(kD, controllerName + "/kD");
            this.veloMax = new TuneableNumber(baseConstraints.maxVelocity, controllerName + "/velocityMax");
            this.accelMax = new TuneableNumber(baseConstraints.maxAcceleration, controllerName + "/accelerationMax");
            this.tolerance = new TuneableNumber(0, controllerName + "/tolerance");

            this.kP.addListener(newNumber -> this.setP(newNumber));
            this.kI.addListener(newNumber -> this.setI(newNumber));
            this.kD.addListener(newNumber -> this.setD(newNumber));
            this.veloMax.addListener(
                newNumber -> super.setConstraints(new Constraints(newNumber, super.getConstraints().maxAcceleration)));
            this.accelMax.addListener(newNumber -> super.setConstraints(
                new Constraints(super.getConstraints().maxVelocity, this.accelMax.get())));
            this.tolerance.addListener(newNumber -> super.setTolerance(this.tolerance.get()));
        }
    }

    public TuneableProfiledPIDController(PIDConstants pidConstants, Constraints baseConstraints,
        String controllerName) {
        this(pidConstants.kP, pidConstants.kI, pidConstants.kD, baseConstraints, controllerName);
    }

    @Override
    public void setConstraints(Constraints constraints) {
        super.setConstraints(constraints);
        if (RuntimeConstants.TuningMode) {
            this.veloMax.set(constraints.maxVelocity);
            this.accelMax.set(constraints.maxAcceleration);
        }
    }

    @Override
    public void setTolerance(double positionTolerance) {
        super.setTolerance(positionTolerance);
        if (RuntimeConstants.TuningMode) {
            this.tolerance.set(positionTolerance);
        }
    }

    /**
     * Calculates the time for this mechanism to acieve its goal.
     */
    public double timeToGetTo(double goal, double current) {
        Constraints constraints = this.getConstraints();
        double absError = Math.abs(goal - current);
        return (absError / constraints.maxVelocity) + (constraints.maxVelocity / constraints.maxAcceleration);
    }

    public double timeToStop() {
        Constraints constraints = this.getConstraints();
        return constraints.maxVelocity / constraints.maxAcceleration;
    }

    @Override
    public double calculate(double measurement) {
        lastMeasure = measurement;
        return super.calculate(measurement);
    }

    @Override
    public double calculate(double measurement, State goal) {
        lastCalculation = super.calculate(measurement, goal);
        return lastCalculation;
    }

    @Override
    public double calculate(double measurement, double goal) {
        lastCalculation = super.calculate(measurement, goal);
        return lastCalculation;
    }

    @Override
    public double calculate(double measurement, State goal, Constraints constraints) {
        lastCalculation = super.calculate(measurement, goal, constraints);
        return lastCalculation;
    }

    /**
     * @return Goal - measure
     */
    public double goalError() {
        return this.getGoal().position - lastMeasure;
    }

    public boolean isHere(double here) {
        System.out.println("isHere: " + here + ", " + lastMeasure + ", " + this.getPositionTolerance());
        return MathUtil.isNear(here, lastMeasure, this.getPositionTolerance());
    }
}
