
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem;

import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

public class VisionSystem {

    protected CameraIO[] cameraIOs;


    public VisionSystem(CameraIO[] cameraIOs) {
        this.cameraIOs = cameraIOs;
    }

    public RawFiducial getTag(int cameraIndex) {
        return cameraIOs[cameraIndex].getFiducialData()[0];
    }

    public RawFiducial[] getTags(int cameraIndex) {
        return cameraIOs[cameraIndex].getFiducialData();
    }

    public PoseEstimate getPoseEstimation(int cameraIndex) {
        return cameraIOs[cameraIndex].getPoseEstimation();
    }

    public void update() {
        for (CameraIO cameraIO : cameraIOs) {
            cameraIO.updatePeriodic();
        }
    }  
  

}
