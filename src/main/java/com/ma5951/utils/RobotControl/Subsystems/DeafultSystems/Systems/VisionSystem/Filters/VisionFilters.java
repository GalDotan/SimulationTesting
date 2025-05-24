
//Linear Velocity
//Angular Velocity
//In field and not in specefied boxes
//Flickering pose 1. stationery flickring 2. distance from odometry (only auto)
//not deafult pose

//Is Flickering detection 

package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem.Filters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Vision.VisionIO;

@SuppressWarnings("static-access")
public class VisionFilters {
    private VisionIO visionIO;
    private VisionFiltersConfig config;
    private Supplier<Pose2d> robotPoSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private Translation2d robotPose;
    private ChassisSpeeds robotSpeeds;
    private Pose2d deafultPose = new Pose2d();
    private Pose2d visionPose;
    private double robotVelocity;

    public VisionFilters(VisionIO VisionIO, VisionFiltersConfig configuration, Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> robotSpeeds) {
        visionIO = VisionIO;
        config = configuration;
        robotPoSupplier = robotPose;
        robotSpeedsSupplier = robotSpeeds;
    }

//     private boolean immediateGate(
//     Pose2d visionPose,
//     Pose2d odometryPose,
//     ChassisSpeeds measuredSpeeds,
//     double dt) {

//   // 1) “Position within 10 cm?”
//   double deltaPos = visionPose.getTranslation()
//                       .getDistance(odometryPose.getTranslation());
//   if (deltaPos > 0.10) {
//     return false; // too big a jump, reject outright
//   }

//   // 2) “Heading within 5°?”
//   double deltaYaw = Math.abs(
//       visionPose.getRotation().minus(odometryPose.getRotation())
//   .getDegrees());
//   if (deltaYaw > 5.0) {
//     return false;
//   }

//   // 3) “Motion inside 1.1× expected?”
//   Translation2d expected = new Translation2d(
//       measuredSpeeds.vxMetersPerSecond * dt,
//       measuredSpeeds.vyMetersPerSecond * dt
//   );
//   double maxAllowed = expected.getNorm() * 1.1;
//   double actualMotion = visionPose.getTranslation()
//                          .minus(lastVisionPose.getTranslation())
//                          .getNorm();
//   if (actualMotion > maxAllowed) {
//     return false;
//   }

//   // If we make it this far, everything lines up *very* well — accept immediately.
//   return true;
// }


    public void updateFilterConfig(VisionFiltersConfig configuration) {
        config = configuration;
    }

    public boolean isValidForGyroReset() {
        return visionIO.getTargetCount() > 1 && visionIO.getRawFiducial().distToCamera < 2
                && visionIO.getRawFiducial().ambiguity < config.AMBIGUITY_FOR_GYRO_RESET &&
                robotSpeeds.vxMetersPerSecond < config.SPEED_FOR_GYRO_RESET &&
                robotSpeeds.vyMetersPerSecond < config.SPEED_FOR_GYRO_RESET &&
                robotSpeeds.omegaRadiansPerSecond < config.SPEED_FOR_GYRO_RESET;
    }

    public boolean isValidForUpdate(Pose2d visionPose2d) {
        visionPose = visionPose2d;
        return inVelocityFilter() && inField() && notInFieldObstacles() && inOdometryRange()
                && shouldUpdateByRobotState() && notDeafultPose() && isVisionMatchingVelocity();
    }

    private boolean inVelocityFilter() {
        robotSpeeds = robotSpeedsSupplier.get();
        return robotSpeeds.vxMetersPerSecond <= config.robotUpdateSpeed.vxMetersPerSecond &&
                robotSpeeds.vyMetersPerSecond <= config.robotUpdateSpeed.vyMetersPerSecond &&
                robotSpeeds.omegaRadiansPerSecond <= config.robotUpdateSpeed.omegaRadiansPerSecond;
    }

    private boolean inField() {
        return config.fieldRectangle.contains(robotPoSupplier.get().getTranslation());
    }

    private boolean notInFieldObstacles() {
        if (config.fieldObstaclesRectangles != null) {
            robotPose = robotPoSupplier.get().getTranslation();
            for (Rectangle2d obstacles : config.fieldObstaclesRectangles) {
                if (obstacles.contains(robotPose)) {
                    return false;
                }
            }
        }

        return true;
    }

    private boolean inOdometryRange() {
        if ((config.visionToOdometryInTeleop && DriverStation.isTeleop()) || DriverStation.isAutonomous()) {
            robotPose = robotPoSupplier.get().getTranslation();
            return robotPose.getDistance(visionPose.getTranslation()) < config.visionToOdometry;
        }
        return true;
    }

    public boolean isFlickering() {
        return isVisionMatchingVelocity();
    }

    private boolean isVisionMatchingVelocity() {
        robotSpeeds = robotSpeedsSupplier.get();
        robotVelocity = (Math.sqrt(Math.pow(robotSpeeds.vxMetersPerSecond, 2) +
        Math.pow(robotSpeeds.vyMetersPerSecond, 2)));
        if (robotVelocity < config.maxVelocityForVisionVelocityFilter) {
            return (robotPoSupplier.get().getTranslation().getDistance(
                    visionPose.getTranslation()) <= robotVelocity * 0.02 + config.VISION_VELOCITY_TOLERANCE);
        }

        return true;
    }

    private boolean shouldUpdateByRobotState() {
        if ((config.updateInAuto && DriverStation.isAutonomous()) || !DriverStation.isAutonomous()) {
            return true;
        }

        return false;
    }

    private boolean notDeafultPose() {
        return visionIO.getEstimatedPose().pose != deafultPose;
    }

}
