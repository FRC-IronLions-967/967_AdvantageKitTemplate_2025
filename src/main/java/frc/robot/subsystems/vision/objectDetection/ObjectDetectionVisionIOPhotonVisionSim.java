package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class ObjectDetectionVisionIOPhotonVisionSim extends ObjectDetectionVisionIOPhotonVision {
  private final VisionSystemSim simSystem;
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> poseSupplier;
  private final TargetModel coral;
  private Pose3d coralPose;
  private VisionTargetSim coralTarget;

  private SimCameraProperties cameraProp;

  public ObjectDetectionVisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera, poseSupplier);
    this.poseSupplier = poseSupplier;

    // Sim system *without* AprilTags
    simSystem = new VisionSystemSim("objectDetectionOnly");

    // create coral
    coral = new TargetModel(0.5, 0.25);
    coralPose = new Pose3d(1, 1, 1, new Rotation3d());
    coralTarget = new VisionTargetSim(coralPose, coral);
    simSystem.addVisionTargets(coralTarget);

    // The simulated camera properties
    cameraProp = new SimCameraProperties();
    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(super.camera, cameraProp);
    simSystem.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    simSystem.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
