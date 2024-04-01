package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;

import java.time.Period;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    //public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    private final LimelightResults results = new LimelightResults();
    private Field2d field = new Field2d();
    
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3));
    private static final Vector<N3> localMesurementsStdDevs = VecBuilder.fill(0.07, 0.07, Units.degreesToRadians(10));
    
    public Swerve() {
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "canivore");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        SmartDashboard.putData(field);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        
        swerveOdometry = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                this.getGyroYaw(),
                this.getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                localMesurementsStdDevs);

        //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                // this::driveRobotRelative, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds

                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        /*
         * AutoBuilder.configureHolonomic(
         * this::getPose, // Robot pose supplier
         * this::setPose, // Method to reset odometry (will be called if your auto has a
         * starting pose)
         * () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
         * // ChassisSpeeds supplier.
         * // MUST BE ROBOT RELATIVE
         * this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE
         * ChassisSpeeds
         * new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
         * likely live in your
         * // Constants class
         * new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
         * new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
         * 4.5, // Max module speed, in m/s
         * 0.4, // Drive base radius in meters. Distance from robot center to furthest
         * module.
         * new ReplanningConfig() // Default path replanning config. See the API for the
         * options here
         * ),
         * () -> {
         * // Boolean supplier that controls when the path will be mirrored for the red
         * // alliance
         * // This will flip the path being followed to the red side of the field.
         * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
         * 
         * var alliance = DriverStation.getAlliance();
         * if (alliance.isPresent()) {
         * return alliance.get() == DriverStation.Alliance.Red;
         * }
         * return false;
         * },
         * this // Reference to this subsystem to set requirements
         * );
         */
    }

    // All other subsystem initialization
    // ...

    // Configure AutoBuilder last
    /*
     * AutoBuilder.configureHolonomic(
     * this::getPose, // Robot pose supplier
     * this::resetPose, // Method to reset odometry (will be called if your auto has
     * a starting pose)
     * this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
     * RELATIVE
     * this::driveRobotRelative, // Method that will drive the robot given ROBOT
     * RELATIVE ChassisSpeeds
     * new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
     * likely live in your
     * // Constants class
     * new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
     * new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
     * 4.5, // Max module speed, in m/s
     * 0.4, // Drive base radius in meters. Distance from robot center to furthest
     * module.
     * new ReplanningConfig() // Default path replanning config. See the API for the
     * options here
     * ),
     * () -> {
     * // Boolean supplier that controls when the path will be mirrored for the red
     * // alliance
     * // This will flip the path being followed to the red side of the field.
     * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
     * 
     * var alliance = DriverStation.getAlliance();
     * if (alliance.isPresent()) {
     * return alliance.get() == DriverStation.Alliance.Red;
     * }
     * return false;
     * },
     * this // Reference to this subsystem to set requirements
     * );
     * }
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void runVelocity(ChassisSpeeds speed) {
        this.setChassisSpeeds(speed, true, true);
    }

    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
        setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        this.targetChassisSpeeds = targetChassisSpeeds;
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        swerveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    


    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        //field.setRobotPose(getPose());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            // SmartDashboard.putData(field);
        }
    
      
    }
}