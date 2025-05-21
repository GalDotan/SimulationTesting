
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;


import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem.Filters.VisionFilters;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem.Filters.VisionFiltersConfig;
import com.ma5951.utils.RobotControl.Utils.Camera.Cameras;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConstants {

    public String cameraName;
    public Cameras camerasType;
    public Transform3d cameraPosition;
    public boolean useMT2;
    public VisionFilters filters;


    public CameraConstants(String cameraName,Cameras camerasType, Transform3d cameraPosition, boolean useMT2, VisionFiltersConfig filtersConfig) {
        this.cameraName = cameraName;
        this.camerasType = camerasType;
        this.cameraPosition = cameraPosition;
        this.useMT2 = useMT2;
        this.filters = new VisionFilters(filtersConfig);
    }

}
