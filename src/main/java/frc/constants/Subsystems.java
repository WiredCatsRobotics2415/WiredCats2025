package frc.constants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Subsystems {
    public class VisionConstants {
        public static final String FirstLL3GName = "limelight-left";
        public static final String SecondLL3GName = "limelight-right";
        public static final String ThirdLL3GName = "limelight-back";

        public static final Matrix<N3, N1> megatag2StdDev = VecBuilder.fill(.7, .7, 9999999);
    }

    public class DriveConstants {
        public static final PIDConstants TranslationPID = new PIDConstants(10, 0, 0);
        public static final PIDConstants RotationPID = new PIDConstants(7, 0, 0);
        public static final PathFollowingController PathFollowingController = new PPHolonomicDriveController(
            TranslationPID, RotationPID);
        public static final PathConstraints DefaultPathConstraints = new PathConstraints(
            MetersPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 2),
            MetersPerSecondPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() / 4),
            RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI / 2));
    }

    public class ElevatorConstants {
        public static final double BaseHeight = 30;
        public static final double Stage2Height = 24;
        public static final double Stage3Height = 23;
        public static final double MaxHeight = 75;
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
