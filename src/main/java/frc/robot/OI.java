package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import frc.constants.Controls.NumpadButtons;
import frc.utils.math.Algebra;
import frc.utils.math.Trig;
import java.util.HashMap;
import java.util.Map;

public class OI {
    CommandXboxController controller;
    CommandJoystick numpad;

    public enum Bind {
        ManualElevatorUp, ManualElevatorDown, ManualArmBack, ManualArmForward, ToggleDealgae, ToggleOuttake,
        ToggleIntake, IntakeFromHPS, IntakeFromGround, AutoScoreLeftL1, AutoScoreLeftL2, AutoScoreLeftL3,
        AutoScoreLeftL4, AutoScoreRightL1, AutoScoreRightL2, AutoScoreRightL3, AutoScoreRightL4, DealgaePreset,
        SeedFieldCentric, StowPreset, ToggleScorePresetsAlignDrive, MinorDriveForward, MinorDriveBackward,
        MinorDriveLeft, MinorDriveRight, AutoIntakeFromGround
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

        binds.put(Bind.ManualElevatorUp, controller.leftBumper());
        binds.put(Bind.ManualElevatorDown, controller.leftTrigger());
        binds.put(Bind.ManualArmForward, controller.rightBumper());
        binds.put(Bind.ManualArmBack, controller.rightTrigger());

        binds.put(Bind.ToggleDealgae, controller.button(GulikitButtons.X));
        binds.put(Bind.ToggleIntake, controller.button(GulikitButtons.B));
        binds.put(Bind.ToggleOuttake, controller.button(GulikitButtons.A));
        binds.put(Bind.AutoIntakeFromGround, controller.button(GulikitButtons.Y));

        binds.put(Bind.MinorDriveForward, controller.povUp());
        binds.put(Bind.MinorDriveBackward, controller.povDown());
        binds.put(Bind.MinorDriveLeft, controller.povLeft());
        binds.put(Bind.MinorDriveRight, controller.povRight());

        binds.put(Bind.IntakeFromGround, numpad.button(NumpadButtons.NumberOne));
        binds.put(Bind.IntakeFromHPS, numpad.button(NumpadButtons.NumberFour));
        binds.put(Bind.DealgaePreset, numpad.button(NumpadButtons.NumberSeven));
        binds.put(Bind.AutoScoreLeftL1, numpad.button(NumpadButtons.NumberTwo));
        binds.put(Bind.AutoScoreLeftL2, numpad.button(NumpadButtons.NumberFive));
        binds.put(Bind.AutoScoreLeftL3, numpad.button(NumpadButtons.NumberEight));
        binds.put(Bind.AutoScoreLeftL4, numpad.button(NumpadButtons.ForwardSlash));
        binds.put(Bind.AutoScoreRightL1, numpad.button(NumpadButtons.NumberThree));
        binds.put(Bind.AutoScoreRightL2, numpad.button(NumpadButtons.NumberSix));
        binds.put(Bind.AutoScoreRightL3, numpad.button(NumpadButtons.NumberNine));
        binds.put(Bind.AutoScoreRightL4, numpad.button(NumpadButtons.Asterisk));
        binds.put(Bind.ToggleScorePresetsAlignDrive, numpad.button(NumpadButtons.Dot));

        binds.put(Bind.StowPreset, numpad.button(NumpadButtons.NumberZero));
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
            double magInitial = Algebra.euclideanDistance(x, y);
            if (Robot.isSimulation()) magInitial = MathUtil.clamp(magInitial, 0, 1);
            double magCurved = Math.pow(deadbandCompensation(magInitial), Controls.CurveExponent);
            double powerCompensated = minimumPowerCompensation(magCurved);
            newX = Trig.cosizzle(angle) * powerCompensated;
            newY = Trig.sizzle(angle) * powerCompensated;
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
