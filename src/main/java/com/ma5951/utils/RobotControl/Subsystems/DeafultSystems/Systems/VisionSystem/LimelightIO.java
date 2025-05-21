
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.CameraConstants;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Transform3d;

public class LimelightIO extends CameraIO {

    private RawFiducial[] fiducialData;
    private PoseEstimate poseEstimation;

    public LimelightIO(CameraConstants cameraIOConstants) {
        super(cameraIOConstants);
    }

    public void setRobotOriantation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(cameraIOConstants.cameraName, yaw, yawRate, pitch, pitchRate, roll,rollRate);
    }

    public void setRobotOriantation(double yaw) {
        LimelightHelpers.SetRobotOrientation(cameraIOConstants.cameraName, yaw, 0, 0, 0, 0,0);
    }

    public void setAllowedIDs(int[] allowedIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraIOConstants.cameraName, allowedIDs);
    }

    public void setDownScale(float downScale) {
       LimelightHelpers.SetFiducialDownscalingOverride(cameraIOConstants.cameraName, downScale);
    }

    public void setCrop(double Xmin, double Xmax, double Ymin, double Ymax) {
        LimelightHelpers.setCropWindow(cameraIOConstants.cameraName, Xmin, Xmax, Ymin, Ymax);
    }

    public void setCameraPosition(Transform3d cameraPosition) {
        LimelightHelpers.setCameraPose_RobotSpace(cameraIOConstants.cameraName, 
        cameraPosition.getY(),
        cameraPosition.getX(), 
        cameraPosition.getZ(), 
        cameraPosition.getRotation().getMeasureX().in(Degrees), 
        cameraPosition.getRotation().getMeasureY().in(Degrees), 
        cameraPosition.getRotation().getMeasureZ().in(Degrees));
    }

    public void setPipline(int pipIndex) {
        LimelightHelpers.setPipelineIndex(cameraIOConstants.cameraName, pipIndex);
    }

    public double getPipline() {
        return LimelightHelpers.getCurrentPipelineIndex(cameraIOConstants.cameraName);
    }

    public RawFiducial[] getFiducialData() {
        fiducialData = LimelightHelpers.getRawFiducials(cameraIOConstants.cameraName);
        if (fiducialData != null && fiducialData.length > 0) {
            return fiducialData;
        }

        return blankFiducialArry;
    }

    public boolean isTag() {
        return LimelightHelpers.getTV(cameraIOConstants.cameraName);
    }

    public PoseEstimate getPoseEstimation() {
        if (cameraIOConstants.useMT2) {
            poseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraIOConstants.cameraName);
        } else {
            poseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraIOConstants.cameraName);
        }

        if (poseEstimation != null) {
            return poseEstimation;
        }

        return new PoseEstimate();
    }

    @Override
    public void updatePeriodic() {
    }

}
