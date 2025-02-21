package frc.subsystems.vision;

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
        limelightCameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(LimelightSpecs.ThreeGDiagnolFOV));
        limelightCameraProps.setCalibError(0.25, 0.08);
        limelightCameraProps.setFPS(20);
        limelightCameraProps.setAvgLatencyMs(60);
        limelightCameraProps.setLatencyStdDevMs(7.5);

        frontLeftSimCam = new PhotonCameraSim(new PhotonCamera("Front Left LL"), limelightCameraProps);
        frontLeftCam = frontLeftSimCam.getCamera();
        frontRightSimCam = new PhotonCameraSim(new PhotonCamera("Front Right LL"), limelightCameraProps);
        frontRightCam = frontLeftSimCam.getCamera();
        backSimCam = new PhotonCameraSim(new PhotonCamera("Back LL"), limelightCameraProps);
        backCam = frontLeftSimCam.getCamera();

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

    // TODO: Maybe don't need to make this realisitic
    // focus on getting the simulation good enough to follow paths and test placement options
    // vision should have a command to track apriltag sightings until it is interrupted and return a result
    // and this class should offer a method like goToNextOption, which updates an internal state
    // that keeps track of height, side, angle, etc
    // then we should have one tuning command in a new visioncharacterization class that runs
    // paths in sim, adjust options, goes again and again
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

        if (currentPipeline == EndEffectorPipeline.DriverView) {
            inputs.endEffectorCameraAveragePixelValue = 255;
        } else {
            Translation2d currentEndEffectorPosition = CommandSwerveDrivetrain.getInstance()
                .getMapleSimSwerveDrivetrain().mapleSimDrive.getSimulatedDriveTrainPose().getTranslation()
                    .plus(RobotMeasurements.EECamOnGround);
            Triangle2d fovTriangle = Triangle2d.isocelesFromPointAndDiagonal(currentEndEffectorPosition,
                LimelightSpecs.TwoPlusMaxObjectDetectionDistance, LimelightSpecs.TwoPlusHorizontalFOV);
            List<Pose3d> allCoralsOnField = SimulatedArena.getInstance().getGamePiecesByType("Coral");

            boolean seesACoral = false;
            Pose2d closestCoral = null;
            double closestCoralDistance = Double.MAX_VALUE;
            for (Pose3d coral : allCoralsOnField) {
                Pose2d coral2d = coral.toPose2d();
                if (fovTriangle.isInside(coral2d.getTranslation())) {
                    seesACoral = true;
                    double thisCoralDistance = coral2d.getTranslation().getDistance(currentEndEffectorPosition);
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
