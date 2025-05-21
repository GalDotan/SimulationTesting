
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.CameraConstants;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Transform3d;

public abstract class CameraIO {

    protected CameraConstants cameraIOConstants;

    protected RawFiducial[] blankFiducialArry = new RawFiducial[] {
            new RawFiducial(
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0)
    };

    protected RawFiducial blankFiducial = new RawFiducial(
            0,
            0,
            0,
            0,
            0,
            0,
            0);

    public CameraIO(CameraConstants cameraIOConstants) {
        this.cameraIOConstants = cameraIOConstants;
    }

    public abstract void setRobotOriantation(double yaw, double yawRate, double pitch, double pitchRate, double roll,
            double rollRate);

    public abstract void setRobotOriantation(double yaw);

    public abstract void setAllowedIDs(int[] allowedIDs);

    public abstract void setDownScale(float downScale);

    public abstract void setCrop(double Xmin, double Xmax, double Ymin, double Ymax);

    public abstract void setCameraPosition(Transform3d cameraPosition);

    public abstract void setPipline(int pipIndex);

    public abstract double getPipline();

    public abstract boolean isTag();

    public abstract RawFiducial[] getFiducialData();

    public abstract PoseEstimate getPoseEstimation();

    public abstract void updatePeriodic();

}
