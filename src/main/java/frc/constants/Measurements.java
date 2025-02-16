package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.constants.Subsystems.DriveAutoConstants;
import java.util.List;

public class Measurements {
    public class RobotMeasurements {
        // Front of the robot: coral scoring side
        public static final Distance BumperLength = Inches.of(3);

        public static final Distance CenterToFrameRadius = Inches.of(21.313);
        public static final Distance CenterToFramePerpendicular = Inches.of(15.401);
        public static final Distance DriveTrainRadius = Inches.of(18.432785);
        public static final Distance DriveTrainTrackWidth = Inches.of(24.625);

        public static final Angle ElevatorTilt = Degrees.of(3.7); // Towards the front

        public static final Mass RobotWeight = Pounds.of(115); // TODO: update with real value
        public static final MomentOfInertia RobotMOI = KilogramSquareMeters.of(RobotWeight.in(Kilograms) *
            (DriveTrainTrackWidth.in(Meters) / 2) * (DriveAutoConstants.HeadingKA / TunerConstants.driveGains.kA));
        public static final ModuleConfig SwerveModuleConfig = new ModuleConfig(TunerConstants.kWheelRadius,
            TunerConstants.kSpeedAt12Volts, 1.542, // TODO: find this with spring test: cof = fs/(m*g)
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
            Rotation2d.fromDegrees(0)); // ID 7
        public static final Pose2d redReefCDApriltag = new Pose2d(new Translation2d(13.474446, 4.745482),
            Rotation2d.fromDegrees(60)); // ID 8
        public static final Pose2d redReefEFApriltag = new Pose2d(new Translation2d(12.643358, 4.745482),
            Rotation2d.fromDegrees(120)); // ID 9
        public static final Pose2d redReefGHApriltag = new Pose2d(new Translation2d(12.227306, 4.0259),
            Rotation2d.fromDegrees(180)); // ID 10
        public static final Pose2d redReefIJApriltag = new Pose2d(new Translation2d(12.643358, 3.306318),
            Rotation2d.fromDegrees(240)); // ID 11
        public static final Pose2d redReefKLApriltag = new Pose2d(new Translation2d(13.474446, 3.306318),
            Rotation2d.fromDegrees(300)); // ID 6

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

        public static final List<Boolean> ReefAlgaeOnTopAlphabeticOrder;
        static {
            ReefAlgaeOnTopAlphabeticOrder = List.of(true, false, true, false, true, false);
        }
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
}
