package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

public class Subsystems {
    public class NavxMXPPorts {
        //https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/
        public static final int Analog0 = 4;
        public static final int Analog1 = 5;
        public static final int Analog2 = 6;
        public static final int Analog3 = 7;
    }

    public class VisionConstants {
        public static final String FrontLeftName = "limelight-left";
        public static final String FrontRightName = "limelight-right";
        public static final String BackCenterName = "limelight-back";

        public static final String[] PoseEstimationLLNames = new String[] { FrontRightName, FrontLeftName,
            BackCenterName };

        public static final Matrix<N3, N1> MegaTag2StdDev = VecBuilder.fill(.7, .7, 9999999);
    }

    public class LEDStripConstants {
        public static final int PortPWM = 9;
        public static final int Length = 60;
        public static final Distance LedSpacing = Meters.of(1 / 120.0);

        public static final Color WestminsterGreen = new Color(30, 72, 47);
    }

    public class DriveAutoConstants {
        public static final PIDConstants PPTranslationPID = new PIDConstants(10, 0, 0); // test 3: kp 1, test 4-: kp 5, test 13-: kp 10, test 15-: kp 7, test 19-: kp 5, test 21: kp 7
        public static final PIDConstants DTTranslationPID = new PIDConstants(5, 0, 0.5);
        public static final PIDConstants RotationPID = new PIDConstants(7, 0, 0);
        public static final PathFollowingController PathFollowingController = new PPHolonomicDriveController(
            PPTranslationPID, RotationPID);
        public static final PathConstraints DefaultPathConstraints = new PathConstraints(
            MetersPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 2),
            MetersPerSecondPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 4),
            RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI / 2));

        public static final double HeadingkP = 3;
        public static final double HeadingkI = 0;
        public static final double HeadingkD = 0.3;
        public static final double HeadingTolerance = 4;
    }

    public class ElevatorConstants {
        public static final int AnalogPotentiometerPort = NavxMXPPorts.Analog0;
        public static final int LeftMotorPort = 9; //(2/14): this is Elevator1
        public static final int RightMotorPort = 10; //(2/14): this is Elevator2

        public static final Voltage PotentiometerMinVolt = Volts.of(0.0d);
        public static final Voltage PotentiometerMaxVolt = Volts.of(0.0d);
        public static final Distance MinHeightInches = Inches.of(0.0d);
        public static final Distance MaxHeightInches = Inches.of(75.0d);

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 0;
        public static final double kD = 0;

        public static final double VelocityMax = 0;
        public static final double AccelerationMax = 0;

        public static final Distance BaseHeight = Inches.of(38);
        public static final Distance Stage2Height = Inches.of(35);
        public static final Distance Stage3Height = Inches.of(34);
    }

    public class ArmConstants {
        // Setting temporary values
        public static final int MotorID = 32;
        public static final int ThroughborePort = 19;

        // 0 degrees: the arm beam is parallel to the elevator
        public static final double ThroughboreZero = 0.0d;
        public static final double ThroughboreMin = 0;
        public static final double ThroughboreMax = 0;
        public static final Angle MaxDegreesBack = Degrees.of(-107);
        public static final Angle MaxDegreesFront = Degrees.of(107);

        public static final MotorOutputConfigs MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final double veloMax = 0;
        public static final double accelMax = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0;
        public static final double kP = 0;
        public static final double kD = 0;

        public static final double ApproximateMassKg = 1.5; // Approximated from CAD at 2/7
        public static final float RotorToArmGearRatio = 50 / 1; // Planetaries amount unknown as of 2/7
        public static final Distance EffectiveLengthInches = Inches.of(28);
        public static final double GoalTolerance = 0.5;

    }

    public class EndEffectorConstants {
        public static final Distance EffectiveDistanceFromElevator = Inches.of(29);
    }

    public class CoralIntakeConstants {
        // Temporary values for constants
        public static final int MotorID = 13;
        public static final int IRSensor = NavxMXPPorts.Analog1;

        public static final double IntakeSpeed = 0.6;
        public static final double OuttakeSpeed = -0.35;

        public static final int IRThreshold = 100; //(2/14) copied from akit2024

        public static final CurrentLimitsConfigs CurrentLimit = new CurrentLimitsConfigs().withSupplyCurrentLimit(40);
        public static final MotorOutputConfigs MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive);
    }
}
