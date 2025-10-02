package frc.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.AlignmentHelpers;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
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
        // public static final TuneableSuperStructureState Level1 = new TuneableSuperStructureState(Inches.of(44.5),
        // Degrees.of(-20), "Level1");
        public static final TuneableSuperStructureState Level1 = new TuneableSuperStructureState(Inches.of(43.3),
            Degrees.of(179.3), "Level1");
        public static final TuneableNumber Level1DriveOffset = new TuneableNumber(13, "Presets/L1Offset");

        public static final TuneableSuperStructureState Level2 = new TuneableSuperStructureState(Inches.of(56.5),
            Degrees.of(180.6), "Level2");
        public static final TuneableSuperStructureState Level2Scoring = new TuneableSuperStructureState(Inches.of(56.5),
            Degrees.of(1), "Level2Scoring");
        public static final TuneableNumber Level2DriveOffset = new TuneableNumber(12, "Presets/L2Offset");

        public static final TuneableSuperStructureState Level3 = new TuneableSuperStructureState(Inches.of(63),
            Degrees.of(-1), "Level3");
        public static final TuneableNumber Level3DriveOffset = new TuneableNumber(12, "Presets/L3Offset");

        public static final TuneableSuperStructureState Level4 = new TuneableSuperStructureState(Inches.of(78.2),
            Degrees.of(-19), "Level4");
        public static final TuneableNumber Level4DriveOffset = new TuneableNumber(15.7, "Presets/L4Offset");

        public static final TuneableSuperStructureState BottomDeAlgae = new TuneableSuperStructureState(Inches.of(37.5),
            Degrees.of(41), "BottomDeAlgae");
        public static final TuneableSuperStructureState TopDeAlgae = new TuneableSuperStructureState(Inches.of(66.5),
            Degrees.of(22.5), "TopDeAlgae");
        public static final TuneableNumber DealgaeDriveOffset = new TuneableNumber(13, "Presets/BottomDADriveOffset");

        public static final TuneableSuperStructureState IntakeFromHPS = new TuneableSuperStructureState(Inches.of(45.1),
            Degrees.of(145.4), "IntakeFromHPS");
        public static final TuneableNumber HPSDriveOffset = new TuneableNumber(6, "Presets/IntakeFromHPS");

        public static final TuneableSuperStructureState GroundIntake = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(179), Degrees.of(CoralIntakeConstants.GroundAngle.get()), "GroundIntake");
        public static final TuneableSuperStructureState KnockOverStack = new TuneableSuperStructureState(Inches.of(0),
            Degrees.of(179), Degrees.of(25), "KnockOverStack");

        public static final TuneableSuperStructureState ProcessorScore = new TuneableSuperStructureState(
            Inches.of(17.75), Degrees.of(41), "Processor");

        public static final TuneableSuperStructureState GroundIntakeAlgae = new TuneableSuperStructureState(
            Inches.of(17.75), Degrees.of(17.675), "GroundIntakeAlgae");

        public static final TuneableSuperStructureState Barge = new TuneableSuperStructureState(Inches.of(78),
            Degrees.of(158), "Barge");

        public static final TuneableSuperStructureState StackIntakeAlgae = new TuneableSuperStructureState(
            Inches.of(27), Degrees.of(17.675), "StackIntakeAlgae");
    }

    public class AlignmentProfiles {
        public static Transform2d LeftAlignmentL4 = new Transform2d(-0.35555, Units.inchesToMeters(5.4),
            Rotation2d.fromDegrees(5.5));
        public static Transform2d RightAlignmentL4 = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(Inches.of(14)).times(-1).in(Meters), Units.inchesToMeters(-2.5),
            Rotation2d.fromDegrees(4));
        public static Transform2d LeftAlignmentL2L3 = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(Inches.of(12)).times(-1).in(Meters), Units.inchesToMeters(5.4),
            Rotation2d.fromDegrees(5.5));
        public static Transform2d RightAlignmentL2L3 = new Transform2d(0.7098, 0.1365,
            Rotation2d.fromDegrees(179.99999));

        public static final Command takeSnapshot(boolean doAdjustAlignmentProfile) {
            return Commands.runOnce(() -> {
                Pose2d lastAlignment = AlignToReef.getLastAlignment();
                Pose2d current = CommandSwerveDrivetrain.getInstance().getState().Pose;
                if (AlignToReef.getLastSetSide() == null || ReefPresetTo.getLastLevelSet() == null) return;
                System.out.println("---------------------------");
                System.out.println("SNAPSHOT: ");
                System.out.println("    Last Level: " + ReefPresetTo.getLastLevelSet().toString());
                System.out.println("    Last Side: " + AlignToReef.getLastSetSide().toString());
                System.out.println("    Tag ID: " + AlignToReef.getLastApriltagIdAlignedTo());
                double xDiff = Units.metersToInches(lastAlignment.getX() - current.getX());
                double yDiff = Units.metersToInches(current.getY() - lastAlignment.getY());
                double rotDiff = current.getRotation().minus(lastAlignment.getRotation()).getDegrees();
                // if (rotDiff > 90) rotDiff = 180 - rotDiff;
                System.out.println("    X (in): " + xDiff);
                System.out.println("    Y (in): " + yDiff);
                System.out.println("    R (deg): " + rotDiff);
                System.out.println(
                    "(these do not include bumper offsets, so you can put these numbers directly into a transform2d)");
                System.out.println("---------------------------");

                if (doAdjustAlignmentProfile) {
                    if (AlignToReef.getLastSetSide().equals(Side.Left)) {
                        AlignToReef.getLeftOffset().set(AlignToReef.getLeftOffset().get() + yDiff);
                        AlignToReef.getLeftAlignRotation().set(AlignToReef.getLeftAlignRotation().get() + rotDiff);
                        if (ReefPresetTo.getLastLevelSet().equals(Level.L4)) {
                            Presets.Level4DriveOffset.set(Presets.Level4DriveOffset.get() + xDiff);
                        } else {
                            Presets.Level3DriveOffset.set(Presets.Level3DriveOffset.get() + xDiff);
                            Presets.Level2DriveOffset.set(Presets.Level2DriveOffset.get() + xDiff);
                        }
                    } else {
                        AlignToReef.getRightOffset().set(AlignToReef.getRightOffset().get() + yDiff);
                        AlignToReef.getRightAlignRotation().set(AlignToReef.getRightAlignRotation().get() + rotDiff);
                        if (ReefPresetTo.getLastLevelSet().equals(Level.L4)) {
                            Presets.Level4DriveOffset.set(Presets.Level4DriveOffset.get() + xDiff);
                        } else {
                            Presets.Level3DriveOffset.set(Presets.Level3DriveOffset.get() + xDiff);
                            Presets.Level2DriveOffset.set(Presets.Level2DriveOffset.get() + xDiff);
                        }
                    }
                }
            });
        }
    }
}
