package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
  private DriveSubsystem drive;
  private double initialDistance;
  private boolean initialized = false;
  private double target;

  public DriveDistanceCommand(DriveSubsystem drive, double distance) {
    this.drive = drive;
    this.addRequirements(drive);

    target = distance;
  }

  @Override
  public void execute() {
    if (!initialized) {
      initialized = true;
      initialDistance = drive.getAverageEncoderDistance();
    }

    drive.arcadeDrive(0.75, 0);
  }

  @Override
  public boolean isFinished() {
    return (drive.getAverageEncoderDistance() - initialDistance) > target;
  }

  @Override
  public void end(boolean interrupted) {
    initialized = false;
    initialDistance = 0;
  }
}
