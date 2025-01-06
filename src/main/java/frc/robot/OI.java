package main.java.frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverControl;
import frc.utils.RobotPreferences;
import java.util.HashMap;
import java.util.Map;

public static class OI {
    private static OI instance;

    CommandXboxController controller;
    CommandJoystick numpad;

    public Map<String, Trigger> binds = new HashMap<String, Trigger>();

    private boolean isCurve;
    private double curve;
     private double slewRate;

    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;

    public final double DEADBAND = 0.05;

    private OI() {
        
    }

    private double deadbandCompensation(double r) {
        return (r - DEADBAND)/(1 - DEADBAND);
    }

    private double minimumPowerCompensation(double r) {
        return r * (1 - DriverControl.MinimumDrivePower) + DriverControl.MinimumDrivePower;
    }

    public TwoDControllerInput getXY() {
        double x = MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND);
        double y = MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND);
        double newX, newY = 0.0d;
        if (isCurve) {
            double angle = Math.atan2(y, x);
            double magInital = Math.sqrt(x*x + y*y);
            double magCurved = Math.pow(deadbandCompensation(magInital), curve);
            double powerCompensated = minimumPowerCompensation(magCurved);
            newX = Math.cos(angle) * powerCompensated;
            newY = Math.sin(angle) * powerCompensated;
        } else {
            newX = xLimiter.calculate(x);
            newY = yLimiter.calculate(y);
        }
        if (Double.isNaN(newX)) newX = 0.0d;
        if (Double.isNaN(newY)) newY = 0.0d;
        return new TwoDControllerInput(newX, newY);
    }

    public double getRotation() {
        double deadbandCompensated = deadbandCompensation(
            MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND));
        if (isCurve) {
            return Math.pow(minimumPowerCompensation(deadbandCompensated), curve);
        } else {
            return minimumPowerCompensation(deadbandCompensated);
        }
    }

    public XboxController getHIDOfController() {
        return controller.getHID();
    }

    public void setPreferences() {
        if (RobotPreferences.getInputFilter()) {
            //Curve
            isCurve = true;
            curve = RobotPreferences.getCurvePower();
            if (curve < 1) curve = 1; //Note: must include because fractional/negative powers will yield uncontrolable results
        } else {
            //Slew
            isCurve = false;
            slewRate = RobotPreferences.getSlewRateLimit();
            if (slewRate < 0) slewRate = 1.0d; //Note: must include because negative rates will yield uncontrolable results
            xLimiter = new SlewRateLimiter(slewRate);
            yLimiter = new SlewRateLimiter(slewRate);
        }

    }
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

}


