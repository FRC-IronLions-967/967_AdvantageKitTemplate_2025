// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.objectDetection;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** Add your docs here. */
public class ObjectDetectionVisionIOPhotonVisionSim extends ObjectDetectionVisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private TargetModel coralModel = new TargetModel(0.4, 0.3);
  private VisionTargetSim[] targets = {
    new VisionTargetSim(new Pose3d(new Translation3d(1, 1, 0), new Rotation3d()), coralModel),
    new VisionTargetSim(new Pose3d(new Translation3d(1, 7, 0), new Rotation3d()), coralModel)
  };

  public ObjectDetectionVisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera, poseSupplier);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
      visionSim.addVisionTargets("DetectedObjects", targets);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d[] objectPoses = new Pose2d[targets.length];
    for (int i = 0; i < targets.length; i++) {
      objectPoses[i] =
          new Pose2d(targets[i].getPose().getX(), targets[i].getPose().getY(), new Rotation2d());
    }
    Logger.recordOutput("Vision/Coral", objectPoses);

    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
