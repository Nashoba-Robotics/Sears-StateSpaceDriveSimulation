package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class AutonomousCommand extends CommandBase {
    
    private RamseteCommand ramseteCommand;
    private DriveSubsystem drive;
    private Trajectory exampleTrajectory ;

	public AutonomousCommand(DriveSubsystem drive, double distance){

            this.drive = drive;

            // Create a voltage constraint to ensure we don't accelerate too fast
            var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.DriveConstants.kDriveKinematics,
                    7);
        
            // Create config for trajectory
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.DriveConstants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);
        
            // An example trajectory to follow.  All units in meters.
            exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at (1, 2) facing the +X direction
                    new Pose2d(1, 2, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(4, 2, new Rotation2d(0)),
                    // Pass config
                    config);
        
            this.ramseteCommand =
                new RamseteCommand(
                    exampleTrajectory,
                    drive::getPose,
                    new RamseteController(
                        Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.DriveConstants.kDriveKinematics,
                    drive::getWheelSpeeds,
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drive::tankDriveVolts,
                    drive);
        
        
            // Run path following command, then stop at the end.
            this.ramseteCommand = ramseteCommand; 
          }
        
          @Override
          public void execute(){
              
			// Reset odometry to starting pose of trajectory.
            drive.resetOdometry(exampleTrajectory.getInitialPose());
            ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
          }
      
        //   @Override
        //   public boolean isFinished(){
        //       return (m_drive.getAverageEncoderDistance() >= (m_initialDistance + m_distance));
        //   }
      
        //   @Override
        //   public void end(boolean interrupted){
        //       boolean onStart = true;
        //   }
      


    }
