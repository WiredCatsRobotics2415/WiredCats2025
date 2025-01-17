package frc.constants;

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
}
