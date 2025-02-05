package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public class Subsystems {
    public class VisionConstants {
        public static final String FrontLeftName = "limelight-left";
        public static final String FrontRightName = "limelight-right";
        public static final String BackCenterName = "limelight-back";

        public static final String[] PoseEstimationLLNames = new String[] { FrontRightName, FrontLeftName,
            BackCenterName };

        public static final Matrix<N3, N1> megatag2StdDev = VecBuilder.fill(.7, .7, 9999999);
    }

    public class LEDStripConstants {
        public static final int PortPWM = 9;
        public static final int Length = 60;
        public static final Distance LedSpacing = Meters.of(1 / 120.0);

        public static final Color WestminsterGreen = new Color(30, 72, 47);
    }

    public class DriveAutoConstants {
        public static final PIDConstants PPTranslationPID = new PIDConstants(10, 0, 0);
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
        public static final double BaseHeight = 30;
        public static final double Stage2Height = 24;
        public static final double Stage3Height = 23;
        public static final double MaxHeight = 75;
        public static final int ANALOG_POT_PORT = 1;
        public static final double KS = 0;
        public static final double KG = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double MIN_VOLT = 0;
        public static final double MAX_VOLT = 1;
        // I don't know what any of these are but they are for PID I think
        public static final double KP = 0;
        public static final double KD = 0;
        public static final double VELO_MAX = 0;
        public static final double ACCEL_MAX = 0;
        public static final int LEFT_MOTOR_PORT = 2;
        public static final int RIGHT_MOTOR_PORT = 3;
    }

    public class ArmConstants {
        // With 0 degrees being up
        public static final double MaxDegreesBack = -135;
        public static final double MaxDegreesFront = 135;
        public static final double EffectiveLengthInches = 12;
    }

    public class EndEffectorConstants {
        public static final double EffectiveLengthInches = 8;
    }
}
