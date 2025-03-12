package frc.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.superstructure.TuneableSuperStructureState;
import frc.utils.tuning.TuneableDistance;

public class Controls {
    public static final double MaxDriveMeterS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRadS = 1.2 * Math.PI;
    public static final double Deadband = 0.05d;
    public static final double MinimumDrivePower = 0.05d;
    public static final boolean UseCurve = true;
    public static final double CurveExponent = 3;

    public static final double RumbleSoftValue = 0.2d;
    public static final double RumbleHardValue = 0.6d;

    public class GulikitButtons {
        public static final int X = 4;
        public static final int Y = 3;
        public static final int B = 1;
        public static final int A = 2;
        public static final int Plus = XboxController.Button.kStart.value;
        public static final int Minus = XboxController.Button.kBack.value;
        public static final int LeftBumper = XboxController.Button.kLeftBumper.value;
        public static final int RightBumper = XboxController.Button.kRightBumper.value;
        public static final int LeftPaddle = 9;
        public static final int RightPaddle = 10;

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
        public static final TuneableSuperStructureState BumpStow = new TuneableSuperStructureState(Inches.of(4),
            Degrees.of(90), CoralIntakeConstants.StowAngle, "Stow");
        public static final TuneableSuperStructureState Stow = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(90), CoralIntakeConstants.StowAngle, "Stow");

        // Positive: coral scoring side
        public static final TuneableSuperStructureState Level1 = new TuneableSuperStructureState(Inches.of(26),
            Degrees.of(-15), "Level1");
        public static final TuneableDistance Level1DriveOffset = new TuneableDistance(Inches.of(-4.05), // These should be negative because they back away
            "Presets/L1Offset");

        public static final TuneableSuperStructureState Level2 = new TuneableSuperStructureState(Inches.of(50),
            Degrees.of(-15), "Level2");
        public static final TuneableDistance Level2DriveOffset = new TuneableDistance(Inches.of(-4.05),
            "Presets/L2Offset");

        public static final TuneableSuperStructureState Level3 = new TuneableSuperStructureState(Inches.of(60),
            Degrees.of(-15), "Level3");
        public static final TuneableDistance Level3DriveOffset = new TuneableDistance(Inches.of(-5.35),
            "Presets/L3Offset");

        public static final TuneableSuperStructureState Level4 = new TuneableSuperStructureState(Inches.of(77),
            Degrees.of(-20), "Level4");
        public static final TuneableDistance Level4DriveOffset = new TuneableDistance(Inches.of(-6.65),
            "Presets/L4Offset");

        public static final TuneableSuperStructureState BottomDeAlgae = new TuneableSuperStructureState(Inches.of(16),
            Degrees.of(0), "BottomDeAlgae");
        public static final TuneableDistance BottomDADriveOffset = new TuneableDistance(Inches.of(-6),
            "Presets/BottomDADriveOffset");

        public static final TuneableSuperStructureState TopDeAlgae = new TuneableSuperStructureState(Inches.of(32),
            Degrees.of(0), "TopDeAlgae");
        public static final TuneableDistance TopDADriveOffset = new TuneableDistance(Inches.of(-6),
            "Presets/TopDADriveOffset");

        public static final TuneableSuperStructureState IntakeFromHPS = new TuneableSuperStructureState(Inches.of(26),
            Degrees.of(192), "IntakeFromHPS");

        public static final TuneableSuperStructureState GroundIntake = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(195), CoralIntakeConstants.GroundAngle.angle(), "GroundIntake");

        public static final TuneableDistance HPSDriveOffset = new TuneableDistance(Inches.of(-2.5),
            "Presets/TopDADriveOffset");
    }
}
