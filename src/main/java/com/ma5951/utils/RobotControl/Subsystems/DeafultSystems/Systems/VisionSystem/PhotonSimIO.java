package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.CameraConstants;
import com.ma5951.utils.Utils.GeomUtil;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Vision.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class PhotonSimIO extends CameraIO {

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final AprilTagFieldLayout tagLayout;
    private final SimCameraProperties cameraProperties;
    private final String cameraName;
    private final PhotonPoseEstimator poseEstimator;
    private Transform3d cameraTransform;
    private int[] allowedIDs = new int[0];
    private PhotonPipelineResult latestResult;
    private List<RawFiducial> fiducials = new ArrayList<>();
    private RawFiducial[] fiducialsArry = new RawFiducial[0];
    private int id;

    public PhotonSimIO(CameraConstants cameraIOConstants) {
        super(cameraIOConstants);
        this.cameraName = cameraIOConstants.cameraName;
        this.camera = new PhotonCamera(cameraName);

        // Camera simulation properties
        this.cameraProperties = new SimCameraProperties();
        this.cameraProperties.setCalibration(
                cameraIOConstants.camerasType.width,
                cameraIOConstants.camerasType.height,
                Rotation2d.fromDegrees(cameraIOConstants.camerasType.fov));
        this.cameraProperties.setFPS(cameraIOConstants.camerasType.simFps);
        this.cameraProperties.setAvgLatencyMs(28);
        this.cameraProperties.setLatencyStdDevMs(5);
        this.cameraProperties.setCalibError(0.25, 0.08);

        // Simulated camera and vision system
        this.cameraSim = new PhotonCameraSim(camera, cameraProperties);
        this.cameraSim.enableProcessedStream(true);
        this.cameraSim.enableRawStream(true);
        this.cameraSim.enableDrawWireframe(true);
        this.cameraSim.setTargetSortMode(PhotonTargetSortMode.Largest);
        this.cameraSim.setMaxSightRange(5);
        this.visionSim = new VisionSystemSim("photonSim");
        this.cameraTransform = cameraIOConstants.cameraPosition;
        visionSim.addCamera(cameraSim, cameraTransform);

        // AprilTag field layout
        this.tagLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
        visionSim.addAprilTags(tagLayout);

        // PhotonPoseEstimator setup
        this.poseEstimator = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransform);

        updatePeriodic();
    }

    @Override
    public void setRobotOriantation(double yaw, double yawRate, double pitch, double pitchRate, double roll,
            double rollRate) {
    }

    @Override
    public void setRobotOriantation(double yaw) {
    }

    @Override
    public void setAllowedIDs(int[] allowedIDs) {
        this.allowedIDs = allowedIDs != null ? allowedIDs.clone() : new int[0];
    }

    @Override
    public void setDownScale(float downScale) {
    }

    @Override
    public void setCrop(double Xmin, double Xmax, double Ymin, double Ymax) {
    }

    @Override
    public void setCameraPosition(Transform3d cameraPosition) {
        this.cameraTransform = cameraPosition;
        poseEstimator.setRobotToCameraTransform(cameraPosition);
    }

    @Override
    public void setPipline(int pipIndex) {
        camera.setPipelineIndex(pipIndex);
    }

    @Override
    public double getPipline() {
        return camera.getPipelineIndex();
    }

    @Override
    public RawFiducial[] getFiducialData() {
        if (!latestResult.hasTargets()) {
            return blankFiducialArry;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            id = target.getFiducialId();
            if (allowedIDs.length == 0 || Arrays.stream(allowedIDs).anyMatch(a -> a == id)) {
                fiducials.add(new RawFiducial(
                        id,
                        target.getYaw(),
                        target.getPitch(),
                        target.getArea(),
                        target.getSkew(),
                        target.getPoseAmbiguity(),
                        target.getBestCameraToTarget().getX()));
            }
        }

        return fiducials.toArray(fiducialsArry);
    }

    @Override
    public boolean isTag() {
        return latestResult.hasTargets();
    }

    @Override
    public PoseEstimate getPoseEstimation() {
        var result = camera.getLatestResult();
        if (!result.hasTargets())
            return new PoseEstimate();

        List<RawFiducial> fiducials = new ArrayList<>();
        double totalArea = 0, totalDist = 0;
        double minYaw = Double.POSITIVE_INFINITY, maxYaw = Double.NEGATIVE_INFINITY;

        for (PhotonTrackedTarget target : latestResult.targets) {
            if (allowedIDs.length == 0 || Arrays.stream(allowedIDs).anyMatch(a -> a == target.fiducialId)) {
                double yaw = target.getYaw();
                minYaw = Math.min(minYaw, yaw);
                maxYaw = Math.max(maxYaw, yaw);
                totalArea += target.getArea();
                totalDist += target.getBestCameraToTarget().getTranslation().getNorm();

                fiducials.add(new RawFiducial(
                        target.fiducialId,
                        yaw,
                        target.getPitch(),
                        target.getArea(),
                        target.getSkew(),
                        target.getPoseAmbiguity(),
                        target.getBestCameraToTarget().getX()));
            }
        }

        if (fiducials.isEmpty())
            return new PoseEstimate();

        double avgArea = totalArea / fiducials.size();
        double avgDist = totalDist / fiducials.size();
        double tagSpan = maxYaw - minYaw;

        Optional<org.photonvision.EstimatedRobotPose> estimateOpt = poseEstimator.update(latestResult);

        Pose2d pose = estimateOpt.map(est -> est.estimatedPose.toPose2d()).orElse(new Pose2d());

        return new PoseEstimate(
                pose,
                result.getTimestampSeconds(),
                (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0,
                fiducials.size(),
                tagSpan,
                avgDist,
                avgArea,
                fiducials.toArray(new RawFiducial[0]),
                false);
    }

    public void updatePeriodic() {
        visionSim.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
        latestResult = cameraSim.process(
                0,
                GeomUtil.toPose3d((SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose()))
                        .plus(VisionConstants.ROBOT_TO_CAMERA),
                tagLayout.getTags().stream()
                        .map(
                                (a) -> new VisionTargetSim(
                                        a.pose, TargetModel.kAprilTag36h11, a.ID))
                        .collect(Collectors.toList()));
        cameraSim.submitProcessedFrame(latestResult);
        List<PhotonPipelineResult> latest = camera.getAllUnreadResults();
        latestResult = latest.get(0);
    }
}
