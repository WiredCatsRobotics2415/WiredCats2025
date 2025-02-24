package frc.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    public static final double MaxDriveMeterS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRadS = 1.2 * Math.PI;
    public static final double MinimumDrivePower = 0.05d;
    public static final double RumbleSoftValue = 0.2d;
    public static final double RumbleHardValue = 0.6d;
    public static final boolean UseCurve = true;
    public static final double CurveExponent = 3;
    public static final double SlewRate = 8; // ie. 8 units/sec, so 0.125 acceleration limit
    public static final double Deadband = 0.05d;
    public static final double MinorAdjustmentSpeedMeterS = 0.25;

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

    public class NumpadButtons {
        public static final int NumberZero = 1;
        public static final int Dot = 2;
        public static final int NumberOne = 3;
        public static final int NumberTwo = 4;
        public static final int NumberThree = 5;
        public static final int NumberFour = 6;
        public static final int NumberFive = 7;
        public static final int NumberSix = 8;
        public static final int NumberSeven = 9;
        public static final int NumberEight = 10;
        public static final int NumberNine = 11;
        public static final int ForwardSlash = 12;
        public static final int Asterisk = 13;
        public static final int Minus = 14;
        public static final int Plus = 15;
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

        public static final Angle BottomAlgaeDescoreAngle = Degrees.of(75);
        public static final Distance BottomAlgaeDescoreHeight = Inches.of(16);
        public static final Angle TopAlgaeDescoreAngle = Degrees.of(75);
        public static final Distance TopAlgaeDescoreHeight = Inches.of(32);

        public static final Angle IntakeFromHPSAngle = Degrees.of(-12);
        public static final Distance IntakeFromHPSHeight = Inches.of(20.6);

        public static final Angle GroundIntakeAngle = Degrees.of(-100);
        public static final Distance GroundIntakeHeight = Inches.of(0);
    }
}
