package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class ObjectDetectionVisionIOPhotonVisionSim extends ObjectDetectionVisionIOPhotonVision {
  private final VisionSystemSim simSystem;
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> poseSupplier;
  private final VisionTargetSim[] objectTargets;

  public ObjectDetectionVisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera, poseSupplier);
    this.poseSupplier = poseSupplier;

    // Sim system *without* AprilTags
    simSystem = new VisionSystemSim("objectDetectionOnly");

    // Define objects to detect (could be coral, cubes, etc.)
    objectTargets =
        new VisionTargetSim[] {
          new VisionTargetSim(new Pose3d(1, 5.0, 0.2, new Rotation3d()), new TargetModel(0.3, 0.3)),
          new VisionTargetSim(new Pose3d(1, 1.0, 0.2, new Rotation3d()), new TargetModel(0.3, 0.3)),
          new VisionTargetSim(
              new Pose3d(-2.0, 0.0, 0.0, new Rotation3d()),
              new TargetModel(0.3, 0.3)) // 2 meters behind robot
        };
    simSystem.addVisionTargets("DetectedObjects", objectTargets);

    // Set up camera
    var props = new SimCameraProperties();
    props.setFPS(30);
    props.setCalibration(640, 480, new Rotation2d(Math.PI / 3));

    cameraSim = new PhotonCameraSim(super.camera, props, null);
    cameraSim.setMinTargetAreaPixels(0); // or something small
    cameraSim.setMaxSightRange(10.0); // meters
    simSystem.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d[] objectPoses = new Pose2d[objectTargets.length];
    for (int i = 0; i < objectTargets.length; i++) {
      objectPoses[i] =
          new Pose2d(
              objectTargets[i].getPose().getX(),
              objectTargets[i].getPose().getY(),
              new Rotation2d());
    }
    Logger.recordOutput("Vision/Coral", objectPoses);

    simSystem.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
