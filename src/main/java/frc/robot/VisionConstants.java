// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
            /*===============================================================================================================================*/
public static class Vision {
        public static final String kCameraName = "photonvision";
        // Cam mounted facing forward, 17 inches forward of center, 11 inches left of center, 8 inches up from ground, and rotated 20 degrees upward 
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-13.5), Units.inchesToMeters(-12.5), Units.inchesToMeters(8.5)), new Rotation3d(0, Units.degreesToRadians(-20), 0));
        
                // Cam mouned facing backwards, 17 inches back of center, 11 inches right of center, 5 inches up from groud, rotated 20 degrees up, and rotated 180 degrees to face backwards
       // public static final Transform3d robotToBackCam = 
          //      new Transform3d(new Translation3d(Units.inchesToMeters(17), Units.inchesToMeters(11), Units.inchesToMeters(8)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
        
                // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //public static final AprilTagFieldLayout kTagLayout =
          //      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
/*===============================================================================================================================*/
}
