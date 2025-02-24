package frc.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class VisionPoseFuser {
    private final double BaseDistrustConstant = 0.7;
    private final double DistanceFromTagScalar = 0;
    private final double DistanceFromCurrentPoseScalar = 0;
    private final double LinearVelocityOfRobot = 0;
    private final double LinearAccelerationOfRobot = 0;
    private final double AngularVelocityOfRobot = 0;
    private final double AngularAccelerationOfRobot = 0;
    private final double PoseLatencyScalar = 0;
    // To disable vision pose fusing: set this to 0
    private final Distance DistanceFromCurrentPoseCutoffThreshold = RobotMeasurements.CenterToFrameRadius;

    private StatusSignal<LinearAcceleration> pigeonLinearAccelX;
    private StatusSignal<LinearAcceleration> pigeonLinearAccelY;
    private StatusSignal<AngularVelocity> pigeonAngularVeloZ;

    private DoubleDifferentiableValue pigeonAngularVeloDDV = new DoubleDifferentiableValue();

    private boolean firstRun = true;
    @Getter
    @Setter private boolean enabled = false;

    private Vision vision = Vision.getInstance();
    private CommandSwerveDrivetrain drivetrain;

    public VisionPoseFuser(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        Pigeon2 pigeon = drivetrain.getPigeon2();
        pigeonLinearAccelX = pigeon.getAccelerationX();
        pigeonLinearAccelY = pigeon.getAccelerationY();
        pigeonAngularVeloZ = pigeon.getAngularVelocityZWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(50, pigeonLinearAccelX, pigeonLinearAccelY, pigeonAngularVeloZ);
        pigeon.optimizeBusUtilization();

        if (RuntimeConstants.TuningMode) {
            TuningModeTab.getInstance().addCommand("Enable Pose Fuser", new InstantCommand(() -> setEnabled(true)));
            TuningModeTab.getInstance().addCommand("Disable Pose Fuser", new InstantCommand(() -> setEnabled(false)));
        }
    }

    public void update(SwerveDriveState state) {
        BaseStatusSignal.refreshAll(pigeonLinearAccelX, pigeonLinearAccelY, pigeonAngularVeloZ);

        double robotAngularVelocityDS = pigeonAngularVeloZ.getValue().in(DegreesPerSecond);
        pigeonAngularVeloDDV.update(robotAngularVelocityDS);
        double robotAngularAccelerationDSS = pigeonAngularVeloDDV.getFirstDerivative();

        vision.sendOrientation(state.Pose.getRotation().getDegrees(), robotAngularVelocityDS);

        if (firstRun) {
            firstRun = false;
            return;
        }

        if (!enabled) return;
        PoseEstimate[] estimates = vision.getPoseEstimates();

        double[] finalXSTDEVs = new double[estimates.length];
        double[] finalYSTDEVs = new double[estimates.length];
        for (int i = 0; i < estimates.length; i++) {
            PoseEstimate estimate = estimates[i];
            if (estimate.tagCount == 0) continue;

            double distanceFromCurrent = state.Pose.getTranslation().getDistance(estimate.pose.getTranslation());

            if (distanceFromCurrent > DistanceFromCurrentPoseCutoffThreshold.in(Meters)) continue;

            double baseStdDev = BaseDistrustConstant + DistanceFromTagScalar * estimate.avgTagDist +
                DistanceFromCurrentPoseScalar * distanceFromCurrent + AngularVelocityOfRobot * robotAngularVelocityDS +
                AngularAccelerationOfRobot * robotAngularAccelerationDSS + PoseLatencyScalar * estimate.latency;

            double xSTDEV = baseStdDev + LinearVelocityOfRobot * state.Speeds.vxMetersPerSecond +
                LinearAccelerationOfRobot * pigeonLinearAccelX.getValueAsDouble();
            finalXSTDEVs[i] = xSTDEV;

            double ySTDEV = baseStdDev + LinearVelocityOfRobot * state.Speeds.vyMetersPerSecond +
                LinearAccelerationOfRobot * pigeonLinearAccelY.getValueAsDouble();
            finalYSTDEVs[i] = ySTDEV;

            drivetrain.addVisionMeasurement(estimate.pose, Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                VecBuilder.fill(xSTDEV, ySTDEV, 9999999));
        }

        Logger.recordOutput("VisionPoseFuser/Enable", enabled);
        Logger.recordOutput("VisionPoseFuser/LinearVelocityOfRobot",
            Algebra.euclideanDistance(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond));
        Logger.recordOutput("VisionPoseFuser/LinearAccelerationOfRobot",
            Algebra.euclideanDistance(pigeonLinearAccelX.getValueAsDouble(), pigeonLinearAccelY.getValueAsDouble()));
        Logger.recordOutput("VisionPoseFuser/AngularVelocity", robotAngularVelocityDS);
        Logger.recordOutput("VisionPoseFuser/AngularAccelerationOfRobot", robotAngularAccelerationDSS);

        for (int i = 0; i < estimates.length; i++) {
            Logger.recordOutput("VisionPoseFuser/" + VisionConstants.PoseEstimationLLNames[i] + "xSTDEV",
                finalXSTDEVs[i]);
            Logger.recordOutput("VisionPoseFuser/" + VisionConstants.PoseEstimationLLNames[i] + "ySTDEV",
                finalYSTDEVs[i]);
        }
    }
}
