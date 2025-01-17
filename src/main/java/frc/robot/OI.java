package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import java.util.HashMap;
import java.util.Map;

public class OI {
    private static OI instance;

    CommandXboxController controller;
    CommandJoystick numpad;

    public Map<String, Trigger> binds = new HashMap<String, Trigger>();

    private SlewRateLimiter xLimiter = new SlewRateLimiter(Controls.SlewRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(Controls.SlewRate);

    public final double DEADBAND = 0.05;

    private double deadbandCompensation(double r) {
        return (r - Controls.Deadband) / (1 - Controls.Deadband);
    }

    private double minimumPowerCompensation(double r) {
        return r * (1 - Controls.MinimumDrivePower) + Controls.MinimumDrivePower;
    }

    public double[] getXY() {
        double x = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickY), Controls.Deadband);
        double y = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickX), Controls.Deadband);
        double newX, newY = 0.0d;
        if (Controls.UseCurve) {
            double angle = Math.atan2(y, x);
            double magInitial = Math.sqrt(x * x + y * y);
            double magCurved = Math.pow(deadbandCompensation(magInitial), Controls.CurveExponent);
            double powerCompensated = minimumPowerCompensation(magCurved);
            newX = Math.cos(angle) * powerCompensated;
            newY = Math.sin(angle) * powerCompensated;
        } else {
            newX = xLimiter.calculate(x);
            newY = yLimiter.calculate(y);
        }
        if (Double.isNaN(newX)) newX = 0.0d;
        if (Double.isNaN(newY)) newY = 0.0d;
        return new double[] { newX, newY };
    }

    public double getRotation() {
        double deadbandCompensated = deadbandCompensation(
            MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.RightJoystickX), Controls.Deadband));
        if (Controls.UseCurve) {
            return Math.pow(minimumPowerCompensation(deadbandCompensated), Controls.CurveExponent);
        } else {
            return minimumPowerCompensation(deadbandCompensated);
        }
    }

    public XboxController getHIDOfController() { return controller.getHID(); }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        controller = new CommandXboxController(0);
        numpad = new CommandJoystick(1);
    }
}
