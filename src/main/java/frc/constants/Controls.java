package frc.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    public static final double MaxDriveMeterS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRadS = Math.PI;
    public static final double MinimumDrivePower = 0.05d;
    public static final double RumbleSoftValue = 0.2d;
    public static final double RumbleHardValue = 0.6d;
    public static final boolean UseCurve = true;
    public static final double CurveExponent = 3;
    public static final double SlewRate = 4; // ie. 4 units/sec, so 0.25 acceleration limit
    public static final double Deadband = 0.05d;

    public class GulikitButtons {
        public static final int X = 4;
        public static final int Y = 3;
        public static final int B = 1;
        public static final int A = 2;
        public static final int Plus = XboxController.Button.kStart.value;
        public static final int Minus = XboxController.Button.kBack.value;
        public static final int LeftBumper = XboxController.Button.kLeftBumper.value;
        public static final int RightBumper = XboxController.Button.kRightBumper.value;

        public static final int LeftJoystickX = XboxController.Axis.kLeftX.value;
        public static final int LeftJoystickY = XboxController.Axis.kLeftY.value;
        public static final int RightJoystickX = XboxController.Axis.kRightX.value;
        public static final int RightJoystickY = XboxController.Axis.kRightY.value;
        public static final int LeftTrigger = XboxController.Axis.kLeftTrigger.value;
        public static final int RightTrigger = XboxController.Axis.kRightTrigger.value;
    }

    public class Presets {
        public static final Angle Level1Angle = Degrees.of(120);
        public static final Distance Level1Height = Inches.of(12);
        public static final Angle Level2Angle = Degrees.of(120);
        public static final Distance Level2Height = Inches.of(24);
        public static final Angle Level3Angle = Degrees.of(120);
        public static final Distance Level3Height = Inches.of(36);
        public static final Angle Level4Angle = Degrees.of(135);
        public static final Distance Level4Height = Inches.of(72);
    }
}
