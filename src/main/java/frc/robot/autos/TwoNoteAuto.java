package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.VoltageConstants;
import frc.robot.commands.IntakeAndKickerCMD;
import frc.robot.commands.MoveArmToHomePOSCMD;
import frc.robot.commands.RunKickerCMD;
import frc.robot.commands.shootFromPodiumSCG;
import frc.robot.commands.shootFromSubSCG;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeAndKickerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TwoNoteAuto extends SequentialCommandGroup {
        public TwoNoteAuto(Swerve s_Swerve, ShooterSubsystem shooterSubsystem, KickerSubsystem kickerSubsystem,
                        ArmSubsystem armSubsystem, IntakeAndKickerSubsystem intakeAndKickerSubsystem) {
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(-.75, -0.01), new Translation2d(-1.0, 0)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(-1.98, 0, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new shootFromSubSCG(shooterSubsystem, armSubsystem, kickerSubsystem) //

                               
                                

                );
        }
}