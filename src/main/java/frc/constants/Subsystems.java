package frc.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Subsystems {
    /** Looking from the BACK of the Motor */
    public static final boolean TalonFXDirectionClockWise = false;
    /** Looking from the BACK of the Motor */
    public static final boolean TalonFXDirectionCounterClockWise = true;

    /** Looking from the BACK of the Motor */
    public static final boolean SparkMaxDirectionClockWise = false;
    /** Looking from the BACK of the Motor */
    public static final boolean SparkMaxDirectionCounterClockWise = true;

    public static final int MotorOverheatWarningC = 80;

    public class ArmConstants {
        public static final int LeftMotorID = 32;
        public static final int RightMotorID = 14;
        public static final int Potentiometer = 4;
        public static final int LimitSwitch = 1;
        public static final int ThroughborePort = 19;

        public static final double kS = 0.238d;
        public static final double kV = 1e-4d; // effectively 0
        public static final double kA = 1e-4d; // effectively 0
        public static final double kG = 0.1d;

        public static final double kP = 0.18d;
        public static final double kD = 0.0033d;

        public static final double veloMax = 50.0d;
        public static final double accelMax = 50.0d;

        public static final double potentiometerMaxVolt = 1.218d;
        public static final double potentiometerMinVolt = 0.005;
        public static final double potentiometerMaxAngle = 105.7d;
        public static final double potentiometerMinAngle = 0.0d; // Min angle of arm

        public static final double ArmLengthMeters = Units.inchesToMeters(30.404);
        public static final double ApproximateMassKg = 12.42;
        public static final float RotorToArmGearRatio = 280 / 1;
        public static final double GoalTolerance = 3;
    }

    public class SwerveConstants {
        public static final PhoenixPIDController HeadingPIDController = new PhoenixPIDController(
            3, 0, 0.3);

        static {
            HeadingPIDController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public static final int HeadingControllerTolerance = 4;
    }

    public class FlywheelConstants {
        public static final int LeftMotorID = 20;
        public static final int RightMotorID = 16;

        public static final Slot0Configs LeftMotorPID = new Slot0Configs().withKV(0.14)
            .withKP(0.25);
        public static final Slot0Configs RightMotorPID = new Slot0Configs().withKV(0.14)
            .withKP(0.2);

        public static final CurrentLimitsConfigs CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30)
            .withSupplyCurrentThreshold(60).withSupplyTimeThreshold(0.1);
        public static final MotorOutputConfigs CoastConfig = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);

        public static final double GoalToleranceRPM = 250;

        public static final double GearRatio = 3.4d;

        public static double rpsToRPM(double rps) {
            return (rps * 60) * GearRatio;
        }

        public static double rpmToRPS(double goalRPM) {
            return (goalRPM / 60) / GearRatio;
        }
    }

    public class IntakeConstants {
        public static final int IntakeMotorID = 18;
        public static final int FlywheelIR = 5;

        public static final CurrentLimitsConfigs CurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentThreshold(40);

        public static final double IntakeSpeed = 0.6;
        public static final double OuttakeSpeed = -0.35;

        public static final double IRThreshold = 100;
    }

    public class ClawConstants {
        public static final int MotorID = 15;

        public static final int CurrentLimit = 30;
        public static final int GearRatio = 12;

        public static final double MoveDistance = 1.25;

        public static final double Ks = 0.14;
        // public static final double Kp = 3.0;
        // public static final double Kd = 0.01;
        public static final double Kp = 1.25;
        public static final double Kd = 0.004;
        public static final double OutputExtrema = 0.83;
    }

    public class VisionConstants {
        public static final String ShooterLimelightName = "limelight-back";
        public static final String IntakeLimelightName = "limelight-intake";

        public static final Vector<N3> stdDevs = VecBuilder.fill(.7, .7, 9999999);
    }
}
