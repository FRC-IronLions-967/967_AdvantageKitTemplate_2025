// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.objectDetection;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class ObjectDetectionVisionIOPhotonVisionSim extends ObjectDetectionVisionIOPhotonVision {

    public ObjectDetectionVisionIOPhotonVisionSim(String name, Transform3d robotToCamera,
            Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera, poseSupplier);
        //TODO Auto-generated constructor stub
    }

}
