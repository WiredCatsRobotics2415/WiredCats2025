package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import java.util.HashMap;
import java.util.Map;

public class OIs {
    public enum Bindings {
        PigeonReset, LowerArm, RaiseArm, FixArm, Amp, ArmDrivePreset, ArmIntakePosition,
        ArmAngle, Intake, AutoIntake, ManualOuttake, ManualIntake, SpinOff, SpinUpToAmp,
        Shoot, ReverseClaw, ShootClose, FixAll
    }

    public abstract static class OI {
        static SendableChooser<Integer> oiChooser;

        static {
            oiChooser = new SendableChooser<Integer>();
            oiChooser.setDefaultOption("Gulikit Controller", 0);
            SmartDashboard.putData("OI", oiChooser);
        }

        /** The binds map of an OI */
        public Map<Bindings, Trigger> binds = new HashMap<Bindings, Trigger>();

        /**
         * Get appropriately scaled translation values, in raw controller units [-1, 1].
         * Array order is [x, y].
         */
        public abstract double[] getXY();

        /** Get appropriately scaled rotation values, in raw controller units [-1, 1] */
        public abstract double getRotation();

        public abstract XboxController getHIDOfController();
    }

    public static class GulikitController extends OI {
        CommandXboxController controller;
        CommandJoystick numpad;

        private SlewRateLimiter xLimiter = new SlewRateLimiter(Controls.SlewRate);
        private SlewRateLimiter yLimiter = new SlewRateLimiter(Controls.SlewRate);

        public GulikitController() {
            controller = new CommandXboxController(0);
            numpad = new CommandJoystick(1);

            // Swerve
            binds.put(Bindings.PigeonReset,
                controller.button(GulikitButtons.Minus, Robot.buttonEventLoop));

            // Arm
            binds.put(Bindings.LowerArm,
                controller.button(GulikitButtons.LeftBumper, Robot.buttonEventLoop));
            binds.put(Bindings.RaiseArm,
                controller.button(GulikitButtons.RightBumper, Robot.buttonEventLoop));
            binds.put(Bindings.Amp, numpad.button(4, Robot.buttonEventLoop));
            binds.put(Bindings.ArmDrivePreset, numpad.button(5, Robot.buttonEventLoop));
            binds.put(Bindings.ArmIntakePosition, numpad.button(6, Robot.buttonEventLoop));
            binds.put(Bindings.ArmAngle, numpad.button(2, Robot.buttonEventLoop));
            binds.put(Bindings.FixArm, numpad.button(3, Robot.buttonEventLoop));

            // Intake
            binds.put(Bindings.Intake,
                controller.button(GulikitButtons.A, Robot.buttonEventLoop));
            binds.put(Bindings.AutoIntake,
                controller.button(GulikitButtons.B, Robot.buttonEventLoop));
            binds.put(Bindings.ManualOuttake, controller
                .axisGreaterThan(GulikitButtons.LeftTrigger, 0.5, Robot.buttonEventLoop));
            binds.put(Bindings.ManualIntake, numpad.button(8, Robot.buttonEventLoop));

            // Flywheels
            binds.put(Bindings.SpinOff, numpad.button(8, Robot.buttonEventLoop));
            binds.put(Bindings.SpinUpToAmp, numpad.button(9, Robot.buttonEventLoop));
            binds.put(Bindings.ShootClose, numpad.button(7, Robot.buttonEventLoop));

            // Claw
            binds.put(Bindings.Shoot,
                controller.axisLessThan(GulikitButtons.RightTrigger, 0.5));
            binds.put(Bindings.ReverseClaw,
                controller.button(GulikitButtons.Plus, Robot.buttonEventLoop));

            binds.put(Bindings.FixAll, numpad.button(1, Robot.buttonEventLoop));
        }

        private double deadbandCompensation(double r) {
            return (r - Controls.Deadband) / (1 - Controls.Deadband);
        }

        private double minimumPowerCompensation(double r) {
            return r * (1 - Controls.MinimumDrivePower) + Controls.MinimumDrivePower;
        }

        public double[] getXY() {
            double x = MathUtil.applyDeadband(
                controller.getRawAxis(GulikitButtons.LeftJoystickY), Controls.Deadband);
            double y = MathUtil.applyDeadband(
                controller.getRawAxis(GulikitButtons.LeftJoystickX), Controls.Deadband);
            double newX, newY = 0.0d;
            if (Controls.UseCurve) {
                double angle = Math.atan2(y, x);
                double magInitial = Math.sqrt(x * x + y * y);
                double magCurved = Math.pow(deadbandCompensation(magInitial),
                    Controls.CurveExponent);
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
            double deadbandCompensated = deadbandCompensation(MathUtil.applyDeadband(
                controller.getRawAxis(GulikitButtons.RightJoystickX), Controls.Deadband));
            if (Controls.UseCurve) {
                return Math.pow(minimumPowerCompensation(deadbandCompensated),
                    Controls.CurveExponent);
            } else {
                return minimumPowerCompensation(deadbandCompensated);
            }
        }

        public XboxController getHIDOfController() { return controller.getHID(); }
    }
}
