package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
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
        public static final String EndEffectorName = "limelight-end";

        public static final String[] PoseEstimationLLNames = new String[] { FrontRightName, FrontLeftName,
            BackCenterName };

        public enum LimelightsForElements {
            Reef(new int[] { 0, 1 }), Processor(new int[] { 0, 1 }), HumanPlayerStation(new int[] { 2 });

            public int[] indexInPEList;

            private LimelightsForElements(int[] indexes) {
                this.indexInPEList = indexes;
            }
        }
    }

    public class LEDStripConstants {
        public static final int PortPWM = 9;
        public static final int BlinkinPort = 13;
        public static final int NumberOfLeds = 5 * 60; // https://www.revrobotics.com/rev-11-1197/
        public static final Distance LedSpacing = Meters.of(1.0 / 60.0);

        public static final TuneableColor WestminsterGreen = new TuneableColor(30, 72, 47, "WestminsterGreen");

        public enum UseableColor {
            BreathingGreen(new Color(30, 72, 47), 0.11), ChasingGreen(new Color(30, 72, 47), 0.01),
            Green(new Color(30, 72, 47), 0.75), Orange(Color.kOrange, 0.65), Purple(Color.kPurple, 0.91),
            Red(Color.kRed, 0.61), Blue(Color.kBlue, 0.87), SkyBlue(Color.kSkyBlue, 0.83), Yellow(Color.kYellow, 0.69),
            White(Color.kWhite, 0.93), Gray(Color.kGray, 0.97), Black(Color.kBlack, 0.99),
            Rainbow(new Color(255, 255, 255), -0.45), Pink(new Color(255, 105, 180), 0.57);

            public Color color;
            public double sparkPWMLevel;

            private UseableColor(Color color, double sparkPWMLevel) {
                this.color = color;
                this.sparkPWMLevel = sparkPWMLevel;
            }
        }
    }

    public class DriveConstants {
        public static final TuneableNumber PPTranslationP = new TuneableNumber(20.0d, "Drive/PPTranslationP");
        public static final TuneableNumber RotationP = new TuneableNumber(15.0d, "Drive/RotationP");

        public static final TuneableNumber BaseVelocityMax = new TuneableNumber(Controls.MaxDriveMeterS,
            "Drive/BaseVelocityMax");
        public static final TuneableNumber BaseXAccelerationMax = new TuneableNumber(
            (1 / 0.25) * Controls.MaxDriveMeterS, "Drive/BaseXAccelerationMax");
        public static final TuneableNumber BaseYAccelerationMax = new TuneableNumber(
            (1 / 0.125) * Controls.MaxDriveMeterS, "Drive/BaseYAccelerationMax");
        public static final TuneableNumber BaseRotationAccelMax = new TuneableNumber(
            (1 / 0.125) * Controls.MaxDriveMeterS, "Drive/BaseRotationAccelMax");

        public static PIDConstants PPTranslationPID = new PIDConstants(PPTranslationP.get(), 0, 0); // test 3: kp 1, test 4-: kp 5, test 13-: kp 10, test 15-: kp 7, test 19-: kp 5, test 21: kp 7
        public static PIDConstants RotationPID = new PIDConstants(RotationP.get(), 0, 0);
        public static PathFollowingController PathFollowingController = new PPHolonomicDriveController(PPTranslationPID,
            RotationPID);
        public static final PathConstraints DefaultPathConstraints = new PathConstraints(TunerConstants.kSpeedAt12Volts,
            TunerConstants.kSpeedAt12Volts.div(2).per(Second), RadiansPerSecond.of(Controls.MaxAngularRadS),
            RadiansPerSecondPerSecond.of(Controls.MaxAngularRadS).div(2));

        public static final PIDConstants DTTranslationPID = new PIDConstants(4, 0, 0.1);

        public static final double HeadingKA = 0.015d; // TODO: find with swerve rotation sysid routine

        public static final double HeadingkP = 3;
        public static final double HeadingkI = 0;
        public static final double HeadingkD = 0.3;
        public static final double HeadingTolerance = 4;
    }

    public class ElevatorConstants {
        public static final int AnalogPotentiometerPort = 0;
        public static final int LeftMotorID = 9; // (2/14): this is Elevator1
        public static final int RightMotorID = 10; // (2/14): this is Elevator2

        public static final double PotentiometerMinVolt = 0.372d;
        public static final double PotentiometerMaxVolt = 1.981d;
        public static final double MinHeight = 0.0d;
        public static final double MaxHeight = 78.0d;

        // Prelim gains: https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.874%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A80%2C%22u%22%3A%22in%22%7D
        public static final double kS = 0.0;
        public static final double kG = 0.6;
        public static final double kV = 0.001;
        public static final double kA = 1e-4;
        public static final double kP = 0.2;
        public static final double kD = 0.03;

        public static final TuneableNumber kGForArm = new TuneableNumber(0.1, "ElevatorFF/kGForArm"); // TODO: the first value should be the kg from SysID when the arm is at 0degrees
        public static final TuneableNumber kGForArmWithAlgae = new TuneableNumber(0.05, "ElevatorFF/kGForArmWithAlgae");

        public static final double BaseVelocityMax = 158d;
        public static final double BaseAccelerationMax = 316d;
        public static final double BaseGoalTolerance = 3d;

        public static final float RotorToArmGearRatio = 5 / 1;
        public static final double BaseHeightM = Units.inchesToMeters(25.75 + 0.5833333333);
        public static final double Stage2HeightM = Units.inchesToMeters(25.75 + 0.5833333333);
        public static final double Stage3HeightM = Units.inchesToMeters(25.75 + 0.5833333333);
    }

    public class ArmConstants {
        public static final int MotorID = 11;
        public static final int ThroughborePort = 0;

        // 0 degrees: the arm rail is parallel with the drivebase on the scoring side (front of robot)
        // 180 degrees: the arm rail is parallel with the drive base on the cintake side (back of robot)
        public static final double ThroughboreMin = 1.13;
        public static final double ThroughboreMax = 0.456;
        public static final double MaxDegreesBack = 200;
        public static final double MinDegreesFront = -20;

        // Prelim gains: https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A12.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A25%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kS = 0.12;
        public static final double kG = 0.58;
        public static final double kV = 0.004;
        public static final double kA = 0.002;
        public static final double kP = 0.05;
        public static final double kD = 0.005;

        public static final TuneableNumber BaseVelocityMax = new TuneableNumber(540, "Arm/BaseVelocityMax");
        public static final TuneableNumber BaseAccelerationMax = new TuneableNumber(1080, "Arm/BaseAccelMax");
        public static final TuneableNumber AlgaeAccelerationMax = new TuneableNumber(360, "Arm/AlgaeAccelMax");
        public static final double GoalTolerance = 6;

        public static final double ApproximateMassKg = 1.814;
        public static final float RotorToArmGearRatio = 25 / 1; // Planetaries amount unknown as of 3/5
        public static final Distance EffectiveLength = Inches.of(28);
    }

    public class EndEffectorConstants {
        public static final int MotorID = 12;
        public static final int IRSensorPort = 1;

        public static final TuneableNumber IntakeCoralSpeed = new TuneableNumber(-0.3, "EndEffector/IntakeCoralSpeed");
        public static final TuneableNumber IntakeAlgaeSpeed = new TuneableNumber(0.8, "EndEffector/IntakeAlgaeSpeed");
        public static final TuneableNumber HoldCoralSpeed = new TuneableNumber(-0.225, "EndEffector/IntakeCoralSpeed");
        public static final TuneableNumber HoldAlgaeSpeed = new TuneableNumber(0.6, "EndEffector/IntakeAlgaeSpeed");
        public static final TuneableNumber OuttakeCoralSpeed = new TuneableNumber(0.65,
            "EndEffector/OuttakeCoralSpeed");
        public static final TuneableNumber OuttakeAlageSpeed = new TuneableNumber(-1, "EndEffector/OuttakeAlageSpeed");

        public static final TuneableNumber IRThreshold = new TuneableNumber(100, "EndEffector/IRThreshold"); // (2/14) copied from akit2024
        public static final TuneableNumber AlgaeIntookCameraThreshold = new TuneableNumber(200,
            "EndEffector/AlgaeIntookCameraThreshold");

        public static final double TorqueMonitorJumpThreshold = 17;
        public static final double TorqueMonitorJumpMagnitude = 8;
        public static final double TorqueMonitorTripTime = 0.325;

        public static final Distance EffectiveDistanceFromElevator = Inches.of(26);
        // end of end effector to carriage
        public static final Distance DistanceFromCarriage = Inches.of(22);
        // Constant difference betweeen angle of arm and angle of end of end effector to the carriage
        public static final Angle AngleFromArmWrtCarraige = Degrees.of(-3.05);
        // Difference between angle of arm and angle of end effector with respect to the ground
        public static final Angle AngleFromArmWrtGround = Degrees.of(14);
    }

    public class ClimberConstants {
        public static int MotorID = 15;
    }

    public class CoralIntakeConstants {
        public static final int PivotMotorID = 13;
        public static final int IntakeMotorID = 14;

        public static final int ThroughborePort = 1;
        public static final double ThroughboreMin = 0.228;
        public static final double ThroughboreMax = 0.52;

        public static final double RotorToArmRatio = 15;
        public static final Distance EffectiveLength = Inches.of(23);

        // recalc: https://www.reca.lc/arm?armMass=%7B%22s%22%3A2.3%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A12.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=70&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kP = 0.0;
        public static final double kD = 0.0;

        public static final double BaseGoalTolerance = 5;

        public static final TuneableNumber BaseVelocityMax = new TuneableNumber(180, "CoralIntake/BaseVelocityMax");;
        public static final TuneableNumber BaseAccelerationMax = new TuneableNumber(360, "CoralIntake/BaseAccelMax");;

        public static final TuneableNumber IntakeSpeed = new TuneableNumber(0.6, "CoralIntake/IntakeSpeed");
        public static final TuneableNumber OuttakeSpeed = new TuneableNumber(-0.35, "CoralIntake/OuttakeSpeed");

        public static final TuneableNumber MaxAngle = new TuneableNumber(100, "CoralIntake/MaxAngle");
        public static final double StowAngle = 70;
        public static final TuneableNumber GroundAngle = new TuneableNumber(0, "CoralIntake/GroundAngle");

        public static final Mass Weight = Pounds.of(2.3);
    }
}
