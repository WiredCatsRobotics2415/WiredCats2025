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
    CommandXboxController controller;
    CommandJoystick numpad;

    public enum Bind {
        ManualElevatorUp, ManualElevatorDown, ManualArmBack, ManualArmForward, ToggleDealgae, ToggleOuttake,
        ToggleIntake, IntakeFromHPS, IntakeFromGround, AutoScoreLeftL1, AutoScoreLeftL2, AutoScoreLeftL3,
        AutoScoreLeftL4, AutoScoreRightL1, AutoScoreRightL2, AutoScoreRightL3, AutoScoreRightL4, DealgaePreset,
        SeedFieldCentric
    }

    public Map<Bind, Trigger> binds = new HashMap<Bind, Trigger>();
    private SlewRateLimiter xLimiter = new SlewRateLimiter(Controls.SlewRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(Controls.SlewRate);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Controls.SlewRate);

    private static OI instance;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        controller = new CommandXboxController(0);
        numpad = new CommandJoystick(1);

        binds.put(Bind.SeedFieldCentric, controller.button(GulikitButtons.Plus));

        binds.put(Bind.ManualElevatorUp, controller.rightTrigger());
        binds.put(Bind.ManualElevatorDown, controller.leftTrigger());
        binds.put(Bind.ManualArmForward, controller.povUp());
        binds.put(Bind.ManualArmBack, controller.povDown());

        binds.put(Bind.ToggleDealgae, controller.button(GulikitButtons.X));
        binds.put(Bind.ToggleIntake, controller.button(GulikitButtons.B));
        binds.put(Bind.ToggleOuttake, controller.button(GulikitButtons.A));

        binds.put(Bind.IntakeFromHPS, numpad.button(1));
        binds.put(Bind.IntakeFromHPS, numpad.button(4));
        binds.put(Bind.DealgaePreset, numpad.button(7));
        binds.put(Bind.AutoScoreLeftL1, numpad.button(2));
        binds.put(Bind.AutoScoreLeftL2, numpad.button(5));
        binds.put(Bind.AutoScoreLeftL3, numpad.button(8));
        binds.put(Bind.AutoScoreLeftL4, numpad.button(10)); // Slash
        binds.put(Bind.AutoScoreRightL1, numpad.button(3));
        binds.put(Bind.AutoScoreRightL2, numpad.button(6));
        binds.put(Bind.AutoScoreRightL3, numpad.button(9));
        binds.put(Bind.AutoScoreRightL4, numpad.button(11)); // Asterisk
    }

    private double deadbandCompensation(double r) {
        return (r - Controls.Deadband) / (1 - Controls.Deadband);
    }

    private double minimumPowerCompensation(double r) {
        return r * (1 - Controls.MinimumDrivePower) + Controls.MinimumDrivePower;
    }

    public double[] getXY() {
        double x = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickX), Controls.Deadband);
        double y = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickY), Controls.Deadband);
        double newX, newY = 0.0d;
        if (Controls.UseCurve) {
            double angle = Math.atan2(y, x);
            double magInitial = Math.sqrt(x * x + y * y);
            if (Robot.isSimulation()) magInitial = MathUtil.clamp(magInitial, 0, 1);
            double magCurved = Math.pow(deadbandCompensation(magInitial), Controls.CurveExponent);
            double powerCompensated = minimumPowerCompensation(magCurved);
            newX = Math.cos(angle) * powerCompensated;
            newY = Math.sin(angle) * powerCompensated;
        }
        if (Double.isNaN(newX)) newX = 0.0d;
        if (Double.isNaN(newY)) newY = 0.0d;
        double rateLimitedX = xLimiter.calculate(x);
        double rateLimitedY = yLimiter.calculate(y);
        return new double[] { rateLimitedX, rateLimitedY };
    }

    public double getRotation() {
        double deadbanded = deadbandCompensation(
            MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.RightJoystickX), Controls.Deadband));
        if (Controls.UseCurve) {
            deadbanded = Math.pow(minimumPowerCompensation(deadbanded), Controls.CurveExponent);
        } else {
            deadbanded = minimumPowerCompensation(deadbanded);
        }
        return rotationLimiter.calculate(deadbanded);
    }

    public XboxController getHIDOfController() { return controller.getHID(); }
}
