package frc.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.Util;
import frc.utils.tuning.TuneableBoolean;
import frc.utils.tuning.TuneableNumber;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    public enum EndEffectorPipeline {
        DriverView, NeuralNetwork
    }

    public enum ObjectRecognized {
        Coral, Algae
    }

    private final TuneableNumber BaseDistrust = new TuneableNumber(0.7, "BaseDistrust");
    private final TuneableNumber DistanceFromCurrentScalar = new TuneableNumber(2, "DistanceFromCurrentScalar");
    private final TuneableNumber DistanceDistrustScalar = new TuneableNumber(15, "DistanceDistrustScalar");
    private final TuneableNumber RotationSpeedDiscardThreshold = new TuneableNumber(720,
        "RotationSpeedDiscardThreshold");
    private final TuneableBoolean AddYawRate = new TuneableBoolean(true, "AddYawRate");

    private Vision() {
        io = (VisionIO) Util.getIOImplementation(VisionIOReal.class, VisionIOSim.class, VisionIO.class);
    }

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    public void sendOrientation(double orientationDegrees, double yawRateDegS) {
        io.setRobotOrientation(orientationDegrees, yawRateDegS);
    }

    public PoseEstimate[] getPoseEstimates() {
        PoseEstimate[] estimates = new PoseEstimate[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < estimates.length; i++) {
            estimates[i] = new PoseEstimate(inputs.poseEstimates[i], inputs.poseTimestampsSeconds[i],
                inputs.poseLatencies[i], inputs.poseTagCounts[i], 0, 0, 0, null, true);
        }

        return estimates;
    }

    /** Averages together the pose from all apriltag limelights and returns that average, with NO ROTATION component */
    public Pose2d getCurrentAveragePose() {
        double averageX = 0.0d, averageY = 0.0d;
        int usedPoses = 0;
        for (int i = 0; i < 3; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageX += inputs.poseEstimates[i].getX();
                averageY += inputs.poseEstimates[i].getY();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return new Pose2d(averageX / usedPoses, averageY / usedPoses, new Rotation2d());
    }

    /**
     * Returns the current average rotation reported by megatag 1. Will return null if no limelights can see any tags.
     */
    public Rotation2d getCurrentAverageRotation() {
        double averageTheta = 0.0d;
        int usedPoses = 0;
        for (int i = 0; i < 3; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageTheta += io.getMT1RotationOf(i).getDegrees();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return Rotation2d.fromDegrees(averageTheta / usedPoses);
    }

    public void setEndEffectorStreamOrientation(boolean upsideDown) {
        io.setEndEffectorStreamOrientation(upsideDown);
    }

    public int getEndEffectorCameraAveragePixelValue() { return inputs.endEffectorCameraAveragePixelValue; }

    public void setEndEffectorPipeline(EndEffectorPipeline pipeline) {
        io.setEndEffectorPipeline(pipeline);
    }

    public boolean objectDetected() {
        return inputs.objectDetected;
    }

    public double getObjectDetectedTx() {
        return inputs.detectedObjectTx;
    }

    public ObjectRecognized getObjectDetectedType() {
        return inputs.detectedObjectLabel == 0 ? ObjectRecognized.Algae : ObjectRecognized.Coral;
    }

    /**
     * Updates a swervedrive with the current global pose estimation. This is in Vision because it does not directly interact with any of the hardware. This should be run periodically.
     *
     * @param driveState    The current state of the drivebase
     * @param driveInstance The instance of the drive
     */
    public void globalPoseEstimation(SwerveDriveState driveState, CommandSwerveDrivetrain driveInstance) {
        Pose2d currentRobotPose = driveState.Pose;
        ChassisSpeeds currentChassisSpeeds = driveState.Speeds;

        if (AddYawRate.get()) {
            this.sendOrientation(currentRobotPose.getRotation().getDegrees(),
                driveInstance.getPigeon2().getAngularVelocityZWorld(true).getValueAsDouble());
        } else {
            this.sendOrientation(currentRobotPose.getRotation().getDegrees(), 0);
        }
        PoseEstimate[] estimates = this.getPoseEstimates();

        for (int i = 0; i < estimates.length; i++) {
            PoseEstimate estimate = estimates[i];
            double twoDDistance = currentRobotPose.getTranslation().getDistance(estimate.pose.getTranslation());
            boolean rejectUpdate = false;
            if (Math.abs(
                (Units.radiansToDegrees(currentChassisSpeeds.omegaRadiansPerSecond))) < RotationSpeedDiscardThreshold
                    .get())
                rejectUpdate = true;
            if (estimate.tagCount > 0) rejectUpdate = true;
            if (twoDDistance > 2 * RobotMeasurements.CenterToFrameRadius.in(Meters)) rejectUpdate = true; // test 1-10: inequality was unintentionally flipped, test 20: added 2x
            if (!rejectUpdate) {
                double xSTDEV = BaseDistrust.get() + (DistanceDistrustScalar.get() * estimate.avgTagDist) +
                    (DistanceFromCurrentScalar.get() * twoDDistance);
                double ySTDEV = BaseDistrust.get() + (DistanceDistrustScalar.get() * estimate.avgTagDist) +
                    (DistanceFromCurrentScalar.get() * twoDDistance);

                driveInstance.addVisionMeasurement(estimate.pose, Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                    VecBuilder.fill(xSTDEV, ySTDEV, 9999999));
            }
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
