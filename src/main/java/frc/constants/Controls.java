package frc.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.commands.AlignToReef;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.superstructure.TuneableSuperStructureState;
import frc.utils.tuning.TuneableNumber;

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
        public static final TuneableSuperStructureState Stow = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(95), Degrees.of(84), "Stow");
        public static final TuneableSuperStructureState AlgaeStow = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(75), "AlgaeStow");
        public static final TuneableSuperStructureState GroundIntakeUp = new TuneableSuperStructureState(Inches.of(30),
            Degrees.of(179), Degrees.of(CoralIntakeConstants.GroundAngle.get()), "GroundIntakeUp");

        // Positive: coral scoring side
        public static final TuneableSuperStructureState Level1 = new TuneableSuperStructureState(Inches.of(28),
            Degrees.of(-20), "Level1");
        public static final TuneableNumber Level1DriveOffset = new TuneableNumber(13, "Presets/L1Offset");
        public static final TuneableNumber Level1OffsetLeft = new TuneableNumber(AlignToReef.getLeftOffset().get(),
            "Presets/L1OffsetLeft");
        public static final TuneableNumber Level1OffsetRight = new TuneableNumber(AlignToReef.getRightOffset().get(),
            "Presets/L1OffsetRight");

        public static final TuneableSuperStructureState Level2 = new TuneableSuperStructureState(Inches.of(46.7),
            Degrees.of(-2), "Level2");
        public static final TuneableNumber Level2DriveOffset = new TuneableNumber(13, "Presets/L2Offset");
        public static final TuneableNumber Level2OffsetLeft = new TuneableNumber(AlignToReef.getLeftOffset().get(),
            "Presets/L2OffsetLeft");
        public static final TuneableNumber Level2OffsetRight = new TuneableNumber(AlignToReef.getRightOffset().get(),
            "Presets/L2OffsetRight");

        public static final TuneableSuperStructureState Level3 = new TuneableSuperStructureState(Inches.of(62),
            Degrees.of(-1), "Level3");
        public static final TuneableNumber Level3DriveOffset = new TuneableNumber(10, "Presets/L3Offset");
        public static final TuneableNumber Level3OffsetLeft = new TuneableNumber(AlignToReef.getLeftOffset().get(),
            "Presets/L3OffsetLeft");
        public static final TuneableNumber Level3OffsetRight = new TuneableNumber(AlignToReef.getRightOffset().get(),
            "Presets/L3OffsetRight");

        public static final TuneableSuperStructureState Level4 = new TuneableSuperStructureState(Inches.of(76.6),
            Degrees.of(-20), "Level4");
        public static final TuneableNumber Level4DriveOffset = new TuneableNumber(28, "Presets/L4Offset");
        public static final TuneableNumber Level4OffsetLeft = new TuneableNumber(AlignToReef.getLeftOffset().get(),
            "Presets/L4OffsetLeft");
        public static final TuneableNumber Level4OffsetRight = new TuneableNumber(AlignToReef.getRightOffset().get(),
            "Presets/L4OffsetRight");

        public static final TuneableSuperStructureState BottomDeAlgae = new TuneableSuperStructureState(Inches.of(37.5),
            Degrees.of(41), "BottomDeAlgae");
        public static final TuneableNumber BottomDADriveOffset = new TuneableNumber(6, "Presets/BottomDADriveOffset");

        public static final TuneableSuperStructureState TopDeAlgae = new TuneableSuperStructureState(Inches.of(66.5),
            Degrees.of(22.5), "TopDeAlgae");
        public static final TuneableNumber TopDADriveOffset = new TuneableNumber(6, "Presets/TopDADriveOffset");

        public static final TuneableSuperStructureState IntakeFromHPS = new TuneableSuperStructureState(Inches.of(45.1),
            Degrees.of(145.4), "IntakeFromHPS");
        public static final TuneableNumber HPSDriveOffset = new TuneableNumber(6, "Presets/IntakeFromHPS");

        public static final TuneableSuperStructureState GroundIntake = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(179), Degrees.of(CoralIntakeConstants.GroundAngle.get()), "GroundIntake");
        public static final TuneableSuperStructureState KnockOverStack = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(179), Degrees.of(25), "KnockOverStack");

        public static final TuneableSuperStructureState ProcessorScore = new TuneableSuperStructureState(Inches.of(0.5),
            Degrees.of(48.7), "Processor");

        public static final TuneableSuperStructureState GroundIntakeAlgae = new TuneableSuperStructureState(
            Inches.of(17.75), Degrees.of(17.675), "GroundIntakeAlgae");
    }
}
