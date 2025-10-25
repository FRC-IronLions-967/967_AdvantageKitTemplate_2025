// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class VisionConstants {
  // April Tag Cmaera 1
  public static final String AprilTagCameraName = "AprilTagCamera";
  public static final Transform3d AprilTagCameraTransform = new Transform3d();
  public static final int AprilTagCameraIndex = 0;

  // Object Detection Camera
  public static final String ObjectDetectionCameraName = "ObjectDetectionCamera";
  public static final Transform3d ObjectDetectionCameraTransform = new Transform3d();
  public static final int ObjectDetectionCameraIndex = 1;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
