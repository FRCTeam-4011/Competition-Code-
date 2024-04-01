// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PoseConfigure {
    
        // Increase these numbers to trust your model's state estimates less.
        public static final double kPositionStdDevX = 0.1;
        public static final double kPositionStdDevY = 0.1;
        public static final double kPositionStdDevTheta = 10;
    
        // Increase these numbers to trust global measurements from vision less.
        public static final double kVisionStdDevX = 2.5;
        public static final double kVisionStdDevY = 2.5;
        public static final double kVisionStdDevTheta = 500;
    
    
}
