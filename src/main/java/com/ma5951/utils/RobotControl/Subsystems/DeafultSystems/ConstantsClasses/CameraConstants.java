
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;


import com.ma5951.utils.RobotControl.Utils.Camera.Cameras;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConstants {

    public String cameraName;
    public Cameras camerasType;
    public Transform3d cameraPosition;
    public boolean useMT2;


    public CameraConstants(String cameraName,Cameras camerasType, Transform3d cameraPosition, boolean useMT2) {
        this.cameraName = cameraName;
        this.camerasType = camerasType;
        this.cameraPosition = cameraPosition;
        this.useMT2 = useMT2;
    }

}
