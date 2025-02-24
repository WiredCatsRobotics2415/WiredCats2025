package frc.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers.PoseEstimate;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
    private VisionSystemSim visionSystemSim;
    private PhotonCameraSim frontLeftSimCam;
    private PhotonCamera frontLeftCam;
    private PhotonCameraSim frontRightSimCam;
    private PhotonCamera frontRightCam;
    private PhotonCameraSim backSimCam;
    private PhotonCamera backCam;

    public VisionIOSim() {
        visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));

        SimCameraProperties limelightCameraProps = new SimCameraProperties();
        limelightCameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(Math.sqrt(80 * 80 + 56 * 56)));
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

        visionSystemSim.addCamera(frontLeftSimCam, new Transform3d());
        visionSystemSim.addCamera(frontRightSimCam, new Transform3d());
        visionSystemSim.addCamera(backSimCam, new Transform3d());
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
        inputs.poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            PoseEstimate estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
            inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];
        }
    }
}
