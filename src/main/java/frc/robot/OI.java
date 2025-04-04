package frc.robot;

import edu.wpi.first.math.MathUtil;
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
        ManualElevatorUp, ManualElevatorDown, ManualArmBack, ManualArmForward, IntakeFromHPS, IntakeFromGround, L1, L2,
        L3, L4, AutoAlignLeft, AutoAlignRight, DealgaePresetTop, SeedFieldCentric, StowPreset, AutoIntakeFromGround,
        ChangeTeleopMode, ProcessorPreset, BargePreset, Shoot, DeAlgae, ManualIntake, DealgaePresetBottom,
        ClimberForward, ClimberBackward, GroundIntakeAlgae, ManualAlgaeShoot, DeStickArm, StackAlageIntake, GroundPound
    }

    public Map<Bind, Trigger> binds = new HashMap<Bind, Trigger>();

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

        binds.put(Bind.ChangeTeleopMode, controller.button(GulikitButtons.RightPaddle));

        binds.put(Bind.ManualElevatorUp, controller.leftBumper());
        binds.put(Bind.ManualElevatorDown, controller.leftTrigger());
        binds.put(Bind.ManualArmForward, controller.rightBumper());
        binds.put(Bind.ManualArmBack, controller.rightTrigger());

        binds.put(Bind.Shoot, controller.button(GulikitButtons.X));
        binds.put(Bind.DeAlgae, controller.button(GulikitButtons.A));
        binds.put(Bind.ManualIntake, controller.button(GulikitButtons.Y));
        binds.put(Bind.ManualAlgaeShoot, controller.button(GulikitButtons.B));

        binds.put(Bind.DeStickArm, controller.povLeft());
        binds.put(Bind.ChangeTeleopMode, controller.button(GulikitButtons.LeftPaddle));
        binds.put(Bind.AutoIntakeFromGround, controller.button(GulikitButtons.RightPaddle));

        binds.put(Bind.StowPreset, numpad.button(NumpadButtons.NumberZero));
        binds.put(Bind.IntakeFromHPS, numpad.button(NumpadButtons.NumberFour));
        binds.put(Bind.DealgaePresetTop, numpad.button(NumpadButtons.NumberSeven));
        binds.put(Bind.DealgaePresetBottom, numpad.button(NumpadButtons.NumberOne));
        binds.put(Bind.L1, numpad.button(NumpadButtons.Dot));
        binds.put(Bind.L2, numpad.button(NumpadButtons.NumberFive));
        binds.put(Bind.L3, numpad.button(NumpadButtons.NumberEight));
        binds.put(Bind.L4, numpad.button(NumpadButtons.ForwardSlash));
        binds.put(Bind.AutoAlignLeft, numpad.button(NumpadButtons.NumberTwo));
        binds.put(Bind.AutoAlignRight, numpad.button(NumpadButtons.NumberThree));
        binds.put(Bind.ProcessorPreset, numpad.button(NumpadButtons.NumberNine));
        binds.put(Bind.GroundIntakeAlgae, numpad.button(NumpadButtons.Plus));
        binds.put(Bind.BargePreset, numpad.button(NumpadButtons.Asterisk));
        binds.put(Bind.ClimberForward, controller.povUp());
        binds.put(Bind.GroundPound, controller.povDown());
        binds.put(Bind.StackAlageIntake, numpad.button(NumpadButtons.Minus));
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
        return new double[] { newX, newY };
    }

    public double[] getRawXY() {
        return new double[] { controller.getRawAxis(GulikitButtons.LeftJoystickX),
            controller.getRawAxis(GulikitButtons.LeftJoystickY) };
    }

    public double getRotation() {
        double deadbanded = deadbandCompensation(
            MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.RightJoystickX), Controls.Deadband));
        if (Controls.UseCurve) {
            deadbanded = Math.pow(minimumPowerCompensation(deadbanded), Controls.CurveExponent);
        } else {
            deadbanded = minimumPowerCompensation(deadbanded);
        }
        return deadbanded;
    }

    public XboxController getHIDOfController() { return controller.getHID(); }
}
