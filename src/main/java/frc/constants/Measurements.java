package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.constants.Subsystems.DriveConstants;
import frc.utils.AllianceDependent;
import java.util.List;

public class Measurements {
    public static final AprilTagFieldLayout ApriltagFieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025Reefscape);

    public class RobotMeasurements {
        // Front of the robot: coral scoring side
        public static final Distance BumperLength = Inches.of(3.204);
        public static final Distance BumperToBumper = Inches.of(36);

        public static final Distance CenterToFrameRadius = Inches.of(21.313);
        public static final Distance CenterToFramePerpendicular = Inches.of(15.401);
        public static final Distance DriveTrainRadius = Inches.of(18.432785);
        public static final Distance DriveTrainTrackWidth = Inches.of(24.625);

        public static final Angle ElevatorTilt = Degrees.of(3.7); // Towards the front

        public static final Mass RobotWeight = Pounds.of(131); // approx, with bumpers and battery
        public static final MomentOfInertia RobotMOI = KilogramSquareMeters.of(RobotWeight.in(Kilograms) *
            (DriveTrainTrackWidth.in(Meters) / 2) * (DriveConstants.HeadingKA / TunerConstants.driveGains.kA));
        public static final ModuleConfig SwerveModuleConfig = new ModuleConfig(TunerConstants.kWheelRadius,
            TunerConstants.kSpeedAt12Volts, 1.542, // TODO: find this with slip current characerization
            DCMotor.getKrakenX60Foc(1), TunerConstants.kSlipCurrent, 1);

        public static final RobotConfig PPRobotConfig = new RobotConfig(RobotWeight, RobotMOI, SwerveModuleConfig,
            new Translation2d[] { new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
                new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
                new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos),
                new Translation2d(TunerConstants.kBackRightYPos, TunerConstants.kBackRightYPos) });
        static {
            System.out.println("PP Robot Config: ");
            System.out.println("    Mass (KG): " + RobotWeight.in(Kilograms));
            System.out.println("    Moi: " + RobotMOI.in(KilogramSquareMeters));
            System.out.println("    Wheel Radius (M): " + TunerConstants.kWheelRadius.in(Meters));
            System.out.println("    Drive Gearing: " + TunerConstants.kDriveGearRatio);
            System.out.println("    True Max Drive Speed: " + TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
            System.out.println("    Wheel COF: " + SwerveModuleConfig.wheelCOF);
            System.out.println("    Drive Motor: " + SwerveModuleConfig.driveMotor);
            System.out.println("    Drive Current Limit: " + TunerConstants.kSlipCurrent.in(Amps));
            System.out.println("    Module Offsets (FL, FR, BL, BR): ");
            for (Translation2d location : PPRobotConfig.moduleLocations) {
                System.out.println("        " + location.toString());
            }
        }

        public static final Transform3d FrontLeftCamera = new Transform3d(Inches.of(2.644), Inches.of(-11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(-20)));
        public static final Transform3d FrontRightCamera = new Transform3d(Inches.of(2.644), Inches.of(11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(20)));
        public static final Transform3d BackCamera = new Transform3d(Inches.of(-7.578), Inches.of(10.052),
            Inches.of(27.982014), new Rotation3d(Degrees.of(0), Degrees.of(3.7 - 32), Degrees.of(-190)));
        public static final Transform3d[] PECameraTransforms = new Transform3d[] { FrontLeftCamera, FrontRightCamera,
            BackCamera };

        public static final Transform2d EECamOnGround = new Transform2d(Inches.of(-11.972173), Inches.of(0),
            Rotation2d.fromDegrees(180));
        public static final double EECamHeightOffGround = 13.82;
        public static final double EECamForward = 1.284;
    }

    public class CoralMeasurements {
        public static final double HeightFromCenterOffGround = 4.5;

        public static final Translation2d LeftBlueHPSDropPosition = new Translation2d(0, 0);
        public static final Translation2d RightBlueHPSDropPosition = new Translation2d(0, 0);
        public static final double HPSDropStdDev = 0;
    }

    public class ReefMeasurements {
        public static final Pose2d blueReefABApriltag = new Pose2d(new Translation2d(3.6576, 4.0259),
            Rotation2d.fromDegrees(0)); // ID 18
        public static final Pose2d blueReefCDApriltag = new Pose2d(new Translation2d(4.073906, 3.306318),
            Rotation2d.fromDegrees(60)); // ID 17
        public static final Pose2d blueReefEFApriltag = new Pose2d(new Translation2d(4.90474, 3.306318),
            Rotation2d.fromDegrees(120)); // ID 22
        public static final Pose2d blueReefGHApriltag = new Pose2d(new Translation2d(5.321046, 4.0259),
            Rotation2d.fromDegrees(180)); // ID 21
        public static final Pose2d blueReefIJApriltag = new Pose2d(new Translation2d(4.90474, 4.745482),
            Rotation2d.fromDegrees(240)); // ID 20
        public static final Pose2d blueReefKLApriltag = new Pose2d(new Translation2d(4.073906, 4.745482),
            Rotation2d.fromDegrees(300)); // ID 19

        public static final Pose2d redReefABApriltag = new Pose2d(new Translation2d(13.890498, 4.0259),
            Rotation2d.fromDegrees(180)); // ID 7
        public static final Pose2d redReefCDApriltag = new Pose2d(new Translation2d(13.474446, 4.745482),
            Rotation2d.fromDegrees(240)); // ID 8
        public static final Pose2d redReefEFApriltag = new Pose2d(new Translation2d(12.643358, 4.745482),
            Rotation2d.fromDegrees(300)); // ID 9
        public static final Pose2d redReefGHApriltag = new Pose2d(new Translation2d(12.227306, 4.0259),
            Rotation2d.fromDegrees(0)); // ID 10
        public static final Pose2d redReefIJApriltag = new Pose2d(new Translation2d(12.643358, 3.306318),
            Rotation2d.fromDegrees(60)); // ID 11
        public static final Pose2d redReefKLApriltag = new Pose2d(new Translation2d(13.474446, 3.306318),
            Rotation2d.fromDegrees(120)); // ID 6

        public static final int blueReefABId = 18;
        public static final int blueReefCDId = 17;
        public static final int blueReefEFId = 22;
        public static final int blueReefGHId = 21;
        public static final int blueReefIJId = 20;
        public static final int blueReefKLId = 19;

        public static final int redReefABId = 7;
        public static final int redReefCDId = 8;
        public static final int redReefEFId = 9;
        public static final int redReefGHId = 10;
        public static final int redReefIJId = 11;
        public static final int redReefKLId = 6;

        public static final List<Pose2d> reefBlueApriltags;
        static {
            reefBlueApriltags = List.of(blueReefABApriltag, blueReefCDApriltag, blueReefEFApriltag, blueReefGHApriltag,
                blueReefIJApriltag, blueReefKLApriltag);
        }

        public static final List<Pose2d> reefRedApriltags;
        static {
            reefRedApriltags = List.of(redReefABApriltag, redReefCDApriltag, redReefEFApriltag, redReefGHApriltag,
                redReefIJApriltag, redReefKLApriltag);
        }

        public static final AllianceDependent<List<Pose2d>> reefApriltagsAlphabetic = new AllianceDependent<List<Pose2d>>(
            reefBlueApriltags, reefRedApriltags);

        public static final int[] reefBlueIds = new int[] { blueReefABId, blueReefCDId, blueReefEFId, blueReefGHId,
            blueReefIJId, blueReefKLId };

        public static final int[] reefRedIds = new int[] { redReefABId, redReefCDId, redReefEFId, redReefGHId,
            redReefIJId, redReefKLId };

        public static final AllianceDependent<int[]> reefIds = new AllianceDependent<int[]>(reefBlueIds, reefRedIds);

        public static final List<Boolean> ReefAlgaeOnTopAlphabeticOrder;
        static {
            ReefAlgaeOnTopAlphabeticOrder = List.of(true, false, true, false, true, false);
        }
    }

    public class HumanPlayerStation {
        public static final Pose2d blueLeftHPS = new Pose2d(new Translation2d(0.8613139999999999, 7.414259999999999),
            Rotation2d.fromDegrees(-54));
        public static final Pose2d blueRightHPS = new Pose2d(new Translation2d(0.8613139999999999, 0.628142),
            Rotation2d.fromDegrees(54));
        public static final Pose2d redLeftHPS = new Pose2d(new Translation2d(16.687292, 0.628142),
            Rotation2d.fromDegrees(-234));
        public static final Pose2d redRightHPS = new Pose2d(new Translation2d(16.687292, 7.414259999999999),
            Rotation2d.fromDegrees(234));

        public static final int blueLeftId = 13;
        public static final int blueRightId = 12;
        public static final int redLeftId = 1;
        public static final int redRightId = 2;

        public static final List<Pose2d> hpsBlueApriltags;
        static {
            hpsBlueApriltags = List.of(blueLeftHPS, blueRightHPS);
        }

        public static final List<Pose2d> hpsRedApriltags;
        static {
            hpsRedApriltags = List.of(redLeftHPS, redRightHPS);
        }

        public static final AllianceDependent<List<Pose2d>> hpsApriltags = new AllianceDependent<List<Pose2d>>(
            hpsBlueApriltags, hpsRedApriltags);

        public static final int[] hpsBlueIds = new int[] { blueLeftId, blueRightId };

        public static final int[] hpsRedIds = new int[] { redLeftId, redRightId };

        public static final AllianceDependent<int[]> hpsIds = new AllianceDependent<int[]>(hpsBlueIds, hpsRedIds);
    }

    public class BalanceConstants {
        public static final double rollThreshold = 0;
        public static final double pitchThreshold = 0;
    }

    public class MotorConstants {
        // The number of seconds to subtract from all times in BreakerCurrentAndTripTimes just to be careful
        public static final double TimeSafetyTolerace = 0.1;
        public static final InterpolatingDoubleTreeMap BreakerCurrentAndTripTimes = new InterpolatingDoubleTreeMap();
        static {
            BreakerCurrentAndTripTimes.put(1.0, 70.0);
            BreakerCurrentAndTripTimes.put(1.5, 10.5 - TimeSafetyTolerace);
            BreakerCurrentAndTripTimes.put(2.0, 4.5 - TimeSafetyTolerace);
            BreakerCurrentAndTripTimes.put(2.5, 1.25 - TimeSafetyTolerace);
        }

        /**
         * Uses the BreakerCurrentAndTripTimes to find the most optimal possible supply lower limit and time
         *
         * @param targetSupply The desired supply current. Can and should be above 40. This will be the maxiumum possible supply current allowance.
         * @param targetStator The desired stator current.
         * @return A CurrentLimitsConfigs object with lower limit and time set optimally.
         */
        public static CurrentLimitsConfigs getCurrentLimitsForSupply(Current targetSupply, Current targetStator) {
            CurrentLimitsConfigs config = new CurrentLimitsConfigs();
            config.SupplyCurrentLimitEnable = true;
            config.SupplyCurrentLimit = targetSupply.in(Amps);
            config.SupplyCurrentLowerLimit = 40;
            config.SupplyCurrentLowerTime = BreakerCurrentAndTripTimes.get(targetSupply.in(Amps) / 40);
            config.StatorCurrentLimitEnable = true;
            config.StatorCurrentLimit = targetStator.in(Amps);
            return config;
        }

        public static final double UniversalTorqueCutoffCurrentSeconds = 45 / 0.02; // No current spike greater than 40 amps in one robot loop, 0.02
    }

    public static class LimelightSpecs {
        public static final double ThreeGFOV = 82;
        public static final double ThreeGVerticalFOV = 56.2;
        public static final double ThreeGDiagnolFOV = 91.144;

        public static final double TwoPlusHorizontalFOV = 62.5;
        public static final double TwoPlusVerticalFOV = 48.9;
        public static final double TwoPlusDiagnolFOV = 2 *
            Math.atan(Math.sqrt(Math.pow(Math.tan(Units.degreesToRadians(TwoPlusHorizontalFOV) / 2), 2) +
                Math.pow(Math.tan(Units.degreesToRadians(TwoPlusVerticalFOV) / 2), 2)));

        public static final double TwoPlusMaxObjectDetectionDistance = 6 * 12;
    }
}
