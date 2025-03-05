package frc.utils.tuning;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.constants.RuntimeConstants;

public class TuneableProfiledPIDController extends ProfiledPIDController {
    private TuneableNumber kP, kI, kD, veloMax, accelMax, tolerance;

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
        this.veloMax.set(constraints.maxVelocity);
        this.accelMax.set(constraints.maxAcceleration);
    }

    @Override
    public void setTolerance(double positionTolerance) {
        super.setTolerance(positionTolerance);
        this.tolerance.set(positionTolerance);
    }
}
