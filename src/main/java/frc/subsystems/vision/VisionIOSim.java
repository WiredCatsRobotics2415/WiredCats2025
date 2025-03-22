package frc.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.Measurements;
import frc.constants.Measurements.LimelightSpecs;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.math.Triangle2d;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
    private EndEffectorPipeline currentPipeline;

    private VisionSystemSim visionSystemSim;
    private PhotonCameraSim frontLeftSimCam;
    private PhotonCamera frontLeftCam;
    private PhotonCameraSim frontRightSimCam;
    private PhotonCamera frontRightCam;
    private PhotonCameraSim backSimCam;
    private PhotonCamera backCam;

    private PhotonCamera[] poseEstimationCameras;
    private PhotonPoseEstimator[] photonEstimators;

    public VisionIOSim() {
        visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));

        SimCameraProperties limelightCameraProps = new SimCameraProperties();
        limelightCameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(LimelightSpecs.ThreeGDiagnolFOV));
        limelightCameraProps.setCalibError(0.25, 0.08);
        limelightCameraProps.setFPS(20);
        limelightCameraProps.setAvgLatencyMs(60);
        limelightCameraProps.setLatencyStdDevMs(7.5);

        frontLeftSimCam = new PhotonCameraSim(new PhotonCamera("Front Left LL"), limelightCameraProps);
        frontLeftCam = frontLeftSimCam.getCamera();
        frontRightSimCam = new PhotonCameraSim(new PhotonCamera("Front Right LL"), limelightCameraProps);
        frontRightCam = frontRightSimCam.getCamera();
        backSimCam = new PhotonCameraSim(new PhotonCamera("Back LL"), limelightCameraProps);
        backCam = backSimCam.getCamera();

        visionSystemSim.addCamera(frontLeftSimCam, RobotMeasurements.FrontLeftCamera);
        visionSystemSim.addCamera(frontRightSimCam, RobotMeasurements.FrontRightCamera);
        visionSystemSim.addCamera(backSimCam, RobotMeasurements.BackCamera);

        photonEstimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotMeasurements.FrontLeftCamera),
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotMeasurements.FrontRightCamera),
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotMeasurements.BackCamera), };
        for (PhotonPoseEstimator estimator : photonEstimators) {
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        poseEstimationCameras = new PhotonCamera[] { frontLeftCam, frontRightCam, backCam };
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d currentRobotPose = CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
            .getSimulatedDriveTrainPose();
        visionSystemSim.update(currentRobotPose);

        inputs.poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < poseEstimationCameras.length; i++) {
            List<PhotonPipelineResult> pipelineResults = poseEstimationCameras[i].getAllUnreadResults();
            if (pipelineResults.size() > 0) {
                PhotonPipelineResult pipelineResult = pipelineResults.get(pipelineResults.size() - 1);
                EstimatedRobotPose estimatedPose = photonEstimators[i].update(pipelineResult).orElse(null);
                if (estimatedPose != null) {
                    inputs.poseEstimates[i] = estimatedPose.estimatedPose.toPose2d();
                    inputs.poseLatencies[i] = pipelineResult.metadata.getLatencyMillis();
                    inputs.poseTimestampsSeconds[i] = estimatedPose.timestampSeconds;
                    inputs.poseTagCounts[i] = estimatedPose.targetsUsed.size();

                    double smallestDistance = Double.MAX_VALUE;
                    int closestApriltagId = -1;
                    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                        if (distance < smallestDistance) {
                            smallestDistance = distance;
                            closestApriltagId = target.fiducialId;
                        }
                    }
                    inputs.poseTagDistances[i] = smallestDistance;
                    inputs.nearestTags[i] = closestApriltagId;

                    continue;
                }
            }
            PoseEstimate estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
            inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];
        }

        Pose2d currentEndEffectorPosition = currentRobotPose.plus(RobotMeasurements.EECamOnGround);
        if (currentPipeline == EndEffectorPipeline.DriverView) {
            Logger.recordOutput("Visualization/VisionIOSim/EETriangle", new Translation2d[] {});
            inputs.endEffectorCameraAveragePixelValue = 255;
        } else {
            Triangle2d fovTriangle = Triangle2d.isocelesFromPointAndDiagonal(
                currentEndEffectorPosition.getTranslation(),
                Inches.of(LimelightSpecs.TwoPlusMaxObjectDetectionDistance),
                Degrees.of(LimelightSpecs.TwoPlusHorizontalFOV), currentEndEffectorPosition.getRotation().getMeasure());
            Logger.recordOutput("Visualization/VisionIOSim/EETriangle",
                new Translation2d[] { fovTriangle.getA(), fovTriangle.getB(), fovTriangle.getC(), fovTriangle.getA() });
            List<Pose3d> allCoralsOnField = SimulatedArena.getInstance().getGamePiecesByType("Coral");

            boolean seesACoral = false;
            Pose2d closestCoral = null;
            double closestCoralDistance = Double.MAX_VALUE;
            for (Pose3d coral : allCoralsOnField) {
                Pose2d coral2d = coral.toPose2d();
                if (fovTriangle.isInside(coral2d.getTranslation())) {
                    seesACoral = true;
                    double thisCoralDistance = coral2d.getTranslation()
                        .getDistance(currentEndEffectorPosition.getTranslation());
                    if (thisCoralDistance < closestCoralDistance) {
                        closestCoral = coral2d;
                    }
                }
            }
            inputs.objectDetected = seesACoral;
            if (seesACoral) {
                inputs.detectedObjectTx = Math.atan2(closestCoral.getY() - currentEndEffectorPosition.getY(),
                    closestCoral.getX() - currentEndEffectorPosition.getX());
                inputs.detectedObjectTy = Math.atan2(closestCoral.getX() - currentEndEffectorPosition.getX(),
                    closestCoral.getY() - currentEndEffectorPosition.getY());
                inputs.detectedObjectLabel = 1;
            }
        }
    }

    @Override
    public void setEndEffectorPipeline(EndEffectorPipeline pipeline) { currentPipeline = pipeline; }
}
