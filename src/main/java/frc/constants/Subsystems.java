package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.subsystems.leds.TuneableColor;
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
    }

    public class LEDStripConstants {
        public static final int PortPWM = 9;
        public static final int BlinkinPort = 13;
        public static final int NumberOfLeds = 5 * 60; // https://www.revrobotics.com/rev-11-1197/
        public static final Distance LedSpacing = Meters.of(1.0 / 60.0);

        public static final TuneableColor WestminsterGreen = new TuneableColor(30, 72, 47, "WestminsterGreen");

        public enum UseableColor {
            BreathingGreen(new Color(30, 72, 47), 0.11), FlashingGreen(new Color(30, 72, 47), 0.01),
            Green(new Color(30, 72, 47), 0.75), Orange(Color.kOrange, 0.65), Purple(Color.kPurple, 0.91),
            Red(Color.kRed, 0.61), Blue(Color.kBlue, 0.87), SkyBlue(Color.kSkyBlue, 0.83), Yellow(Color.kYellow, 0.69),
            White(Color.kWhite, 0.93), Gray(Color.kGray, 0.97), Black(Color.kBlack, 0.99),
            Rainbow(new Color(255, 255, 255), -0.45);

            public Color color;
            public double sparkPWMLevel;

            private UseableColor(Color color, double sparkPWMLevel) {
                this.color = color;
                this.sparkPWMLevel = sparkPWMLevel;
            }
        }
    }

    public class DriveAutoConstants {
        public static final TuneableNumber PPTranslationP = new TuneableNumber(20.0d, "DriveAuto/PPTranslationP");
        public static final TuneableNumber RotationP = new TuneableNumber(15.0d, "DriveAuto/RotationP");

        public static final double BaseVelocityMax = Controls.MaxDriveMeterS;
        public static final double BaseAccelerationMax = 2 * Controls.MaxDriveMeterS;

        public static PIDConstants PPTranslationPID = new PIDConstants(PPTranslationP.get(), 0, 0); // test 3: kp 1, test 4-: kp 5, test 13-: kp 10, test 15-: kp 7, test 19-: kp 5, test 21: kp 7
        public static PIDConstants RotationPID = new PIDConstants(RotationP.get(), 0, 0);
        public static PathFollowingController PathFollowingController = new PPHolonomicDriveController(PPTranslationPID,
            RotationPID);
        public static final PathConstraints DefaultPathConstraints = new PathConstraints(
            MetersPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 2),
            MetersPerSecondPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 4),
            RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI / 2));

        public static final PIDConstants DTTranslationPID = new PIDConstants(0.5, 0, 0);
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
        public static final Voltage PotentiometerMaxVolt = Volts.of(3.3d);
        public static final Distance MinHeight = Inches.of(0.0d);
        public static final Distance MaxHeight = Inches.of(79.0d);

        // Prelim gains: https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.874%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A80%2C%22u%22%3A%22in%22%7D
        public static final double kS = 0.025;
        public static final double kG = 0.11;
        public static final double kV = 0.01;
        public static final double kA = 0.0;
        public static final double kP = 0.2;
        public static final double kD = 0.02;

        public static final TuneableNumber kGForArm = new TuneableNumber(0.11 - kG, "ElevatorFF/kGForArm"); // TODO: the first value should be the kg from SysID when the arm is at 0degrees

        public static final double BaseVelocityMax = 39.5d;
        public static final double BaseAccelerationMax = 79d;
        public static final double BaseGoalTolerance = .5d;

        public static final float RotorToArmGearRatio = 5 / 1;
        public static final Distance BaseHeight = Inches.of(25.75 + 0.5833333333);
        public static final Distance Stage2Height = Inches.of(25.75 + 0.5833333333);
        public static final Distance Stage3Height = Inches.of(25.75 + 0.5833333333);
    }

    public class ArmConstants {
        public static final int MotorID = 11;
        public static final int ThroughborePort = 19;

        // 0 degrees: the arm rail is parallel with the drivebase on the scoring side (front of robot)
        // 180 degrees: the arm rail is with the drive base on the cintake side (back of robot)
        public static final double ThroughboreMin = 0;
        public static final double ThroughboreMax = 1;
        public static final Angle MaxDegreesBack = Degrees.of(210);
        public static final Angle MinDegreesFront = Degrees.of(-30);

        public static final MotorOutputConfigs MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive);

        // Prelim gains: https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A12.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A25%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kS = 0.025;
        public static final double kG = 0.5;
        public static final double kV = 0.1;
        public static final double kA = 0.0;
        public static final double kP = 0.7;
        public static final double kD = 0.01;

        public static final double BaseVelocityMax = 180; // (2/18) copied from akit2024
        public static final double BaseAccelerationMax = 360; // (2/18) copied from akit2024
        public static final Angle GoalTolerance = Degrees.of(0.5);

        public static final double ApproximateMassKg = 1.814;
        public static final float RotorToArmGearRatio = 25 / 1; // Planetaries amount unknown as of 3/5
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
        public static final int PivotMotorId = 13;
        public static final int IntakeMotorId = 14;

        public static final int ThroughborePort = 18;
        public static final double ThroughboreZero = 0;
        public static final double ThroughboreMin = 0;
        public static final double ThroughboreMax = 1;

        public static final double RotorToArmRatio = 15;
        public static final Distance EffectiveLength = Inches.of(20);

        // recalc: https://www.reca.lc/arm?armMass=%7B%22s%22%3A2.3%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A12.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=70&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kS = 0.01;
        public static final double kG = 0.55;
        public static final double kV = 0.29;
        public static final double kA = 0.04;
        public static final double kP = 0.2;
        public static final double kD = 0.02;

        public static final double BaseGoalTolerance = 1;

        public static final double BaseVelocityMax = 180;
        public static final double BaseAccelerationMax = 360;

        public static final double IntakeSpeed = 0.6;
        public static final double OuttakeSpeed = -0.35;

        public static final Angle MaxAngle = Degrees.of(100);
        public static final Angle StowAngle = Degrees.of(70);
        public static final Angle GroundAngle = Degrees.of(0);

        public static final Mass Weight = Pounds.of(2.3);
    }
}
