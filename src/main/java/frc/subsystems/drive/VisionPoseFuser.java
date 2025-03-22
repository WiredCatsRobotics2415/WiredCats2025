package frc.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;

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
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class VisionPoseFuser {
    private final TuneableNumber BaseDistrustConstant = new TuneableNumber(0.5, "VPF/BaseDistrustConstant");
    private final TuneableNumber DistanceFromTagScalar = new TuneableNumber(1, "VPF/DistanceFromTagScalar");
    private final TuneableNumber DistanceFromCurrentPoseScalar = new TuneableNumber(1,
        "VPF/DistanceFromCurrentPoseScalar");
    private final TuneableNumber LinearVelocityOfRobot = new TuneableNumber(1, "VPF/LinearVelocityOfRobot");
    private final TuneableNumber LinearAccelerationOfRobot = new TuneableNumber(4, "VPF/LinearAccelerationOfRobot");
    private final TuneableNumber AngularVelocityOfRobot = new TuneableNumber(3, "VPF/AngularVelocityOfRobot");
    private final TuneableNumber AngularAccelerationOfRobot = new TuneableNumber(6, "VPF/AngularAccelerationOfRobot");
    private final TuneableNumber PoseLatencyScalar = new TuneableNumber(0.075, "VPF/PoseLatencyScalar");
    // To disable vision pose fusing: set this to 0
    private final Distance DistanceFromCurrentPoseCutoffThreshold = RobotMeasurements.CenterToFrameRadius;

    private StatusSignal<LinearAcceleration> pigeonLinearAccelX;
    private StatusSignal<LinearAcceleration> pigeonLinearAccelY;
    private StatusSignal<AngularVelocity> pigeonAngularVeloZ;

    private DoubleDifferentiableValue pigeonAngularVeloDDV = new DoubleDifferentiableValue();

    private boolean firstRun = true;
    @Getter
    @Setter private boolean enabled = true;
    private double robotAngularVelocityDS;
    private double robotAngularAccelerationDSS;

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

    public void sendLimelightsOrientation(SwerveDriveState state) {
        BaseStatusSignal.refreshAll(pigeonLinearAccelX, pigeonLinearAccelY, pigeonAngularVeloZ);

        robotAngularVelocityDS = pigeonAngularVeloZ.getValue().in(DegreesPerSecond);
        pigeonAngularVeloDDV.update(robotAngularVelocityDS);
        robotAngularAccelerationDSS = pigeonAngularVeloDDV.getFirstDerivative();

        vision.sendOrientation(state.Pose.getRotation().getDegrees(), robotAngularVelocityDS);
    }

    public void update(SwerveDriveState state) {
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

            // if (distanceFromCurrent > DistanceFromCurrentPoseCutoffThreshold.in(Meters)) continue;

            double baseStdDev = BaseDistrustConstant.get() + DistanceFromTagScalar.get() * estimate.avgTagDist +
                DistanceFromCurrentPoseScalar.get() * distanceFromCurrent +
                AngularVelocityOfRobot.get() * robotAngularVelocityDS +
                AngularAccelerationOfRobot.get() * robotAngularAccelerationDSS +
                PoseLatencyScalar.get() * estimate.latency;

            double xSTDEV = baseStdDev + LinearVelocityOfRobot.get() * state.Speeds.vxMetersPerSecond +
                LinearAccelerationOfRobot.get() * pigeonLinearAccelX.getValueAsDouble();
            finalXSTDEVs[i] = xSTDEV;

            double ySTDEV = baseStdDev + LinearVelocityOfRobot.get() * state.Speeds.vyMetersPerSecond +
                LinearAccelerationOfRobot.get() * pigeonLinearAccelY.getValueAsDouble();
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
