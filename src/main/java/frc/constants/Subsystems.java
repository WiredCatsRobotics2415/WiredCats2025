package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.tuning.TuneableNumber;

public class Subsystems {
    public class NavxMXPPorts {
        // https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/
        public static final int Analog0 = 4;
        public static final int Analog1 = 5;
        public static final int Analog2 = 6;
        public static final int Analog3 = 7;
    }

    public class VisionConstants {
        public static final String FrontLeftName = "limelight-left";
        public static final String FrontRightName = "limelight-right";
        public static final String BackCenterName = "limelight-back";
        public static final String EndEffectorName = "limelight-endeffector";

        public static final String[] PoseEstimationLLNames = new String[] { FrontRightName, FrontLeftName,
            BackCenterName };
        public static final int[] ReefFacingLLs = new int[] { 0, 1 };

        public static final Matrix<N3, N1> MegaTag2StdDev = VecBuilder.fill(.7, .7, 9999999);
    }

    public class LEDStripConstants {
        public static final int PortPWM = 9;
        public static final int Length = 60;
        public static final Distance LedSpacing = Meters.of(1 / 120.0);

        public static final Color WestminsterGreen = new Color(30, 72, 47);
    }

    public class DriveAutoConstants {
        public static final TuneableNumber PPTranslationP = new TuneableNumber(20.0d, "DriveAuto/PPTranslationP");
        public static final TuneableNumber RotationP = new TuneableNumber(15.0d, "DriveAuto/RotationP");
        public static final TuneableNumber DTTranslationP = new TuneableNumber(0.5, "DriveAuto/DTTranslationP");

        public static PIDConstants PPTranslationPID = new PIDConstants(PPTranslationP.get(), 0, 0); // test 3: kp 1, test 4-: kp 5, test 13-: kp 10, test 15-: kp 7, test 19-: kp 5, test 21: kp 7
        public static PIDConstants DTTranslationPID = new PIDConstants(DTTranslationP.get(), 0, 0);
        public static PIDConstants RotationPID = new PIDConstants(RotationP.get(), 0, 0);
        public static PathFollowingController PathFollowingController = new PPHolonomicDriveController(PPTranslationPID,
            RotationPID);
        public static final PathConstraints DefaultPathConstraints = new PathConstraints(
            MetersPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 2),
            MetersPerSecondPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 4),
            RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI / 2));

        public static final double HeadingKA = 0.015d; // TODO: find with swerve rotation sysid routine

        public static final double HeadingkP = 3;
        public static final double HeadingkI = 0;
        public static final double HeadingkD = 0.3;
        public static final double HeadingTolerance = 4;
    }

    public class ElevatorConstants {
        public static final int AnalogPotentiometerPort = NavxMXPPorts.Analog0;
        public static final int LeftMotorPort = 9; // (2/14): this is Elevator1
        public static final int RightMotorPort = 10; // (2/14): this is Elevator2

        public static final Voltage PotentiometerMinVolt = Volts.of(0.0d);
        public static final Voltage PotentiometerMaxVolt = Volts.of(0.0d);
        public static final Distance MinHeight = Inches.of(0.0d);
        public static final Distance MaxHeight = Inches.of(69.0d);

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 1e-4;
        public static final double kP = 0;
        public static final double kD = 0;

        public static final TuneableNumber VelocityMax = new TuneableNumber(50.0d, "Elevator/VelocityMax"); // (2/18) copied from akit2024
        public static final TuneableNumber AccelerationMax = new TuneableNumber(50.0d, "Elevator/AccelMax"); // (2/18) copied from akit2024
        public static final TuneableNumber GoalTolerance = new TuneableNumber(0.5, "Elevator/GoalTolerance");

        public static final float RotorToArmGearRatio = 5 / 1; // Planetaries amount unknown as of 2/7
        public static final Distance BaseHeight = Inches.of(38);
        public static final Distance Stage2Height = Inches.of(35);
        public static final Distance Stage3Height = Inches.of(34);
    }

    public class ArmConstants {
        public static final int MotorID = 11;
        public static final int ThroughborePort = 19;

        // 0 degrees: the arm beam is parallel to the elevator
        // positive: front of the robot: coral scoring side
        public static final double ThroughboreZero = 0.0d;
        public static final double ThroughboreMin = 0;
        public static final double ThroughboreMax = 0;
        public static final Angle MaxDegreesBack = Degrees.of(-135); // 107 is safe
        public static final Angle MaxDegreesFront = Degrees.of(135);

        public static final MotorOutputConfigs MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0;
        public static final double kP = 0;
        public static final double kD = 0;

        public static final double VelocityMax = 50.0d; // (2/18) copied from akit2024
        public static final double AccelerationMax = 50.0d; // (2/18) copied from akit2024
        public static final double GoalTolerance = 0.5;

        public static final double ApproximateMassKg = 1.5; // Approximated from CAD at 2/7
        public static final float RotorToArmGearRatio = 15 / 1; // Planetaries amount unknown as of 2/7
        public static final Distance EffectiveLength = Inches.of(28);
    }

    public class EndEffectorConstants {
        public static final int MotorID = 12;
        public static final int IRSensorPort = NavxMXPPorts.Analog1;

        public static final double IntakeCoralSpeed = 0.6;
        public static final double IntakeAlgaeSpeed = -0.8;
        public static final double OuttakeCoralSpeed = -0.35;
        public static final double OuttakeAlageSpeed = 0.35;

        public static final int IRThreshold = 100; // (2/14) copied from akit2024
        public static final int AlgaeIntookCameraThreshold = 200;

        public static final Distance EffectiveDistanceFromElevator = Inches.of(29);
    }

    public class CoralIntakeConstants {
        public static final int MotorID = 13;

        public static final double IntakeSpeed = 0.6;
        public static final double OuttakeSpeed = -0.35;
    }
}