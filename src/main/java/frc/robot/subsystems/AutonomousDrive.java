package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousDrive extends CommandBase{

    private double m_distance;
    private double m_initialDistance;
    private DriveSubsystem m_drive;
    private boolean onStart = true;

    public AutonomousDrive(DriveSubsystem drive, double distance){
        
        m_distance = distance;
        m_drive = drive;
        this.addRequirements(m_drive);
    }

    @Override
    public void execute(){
        if(onStart){
            m_initialDistance = m_drive.getAverageEncoderDistance();
            onStart = false;
        }
        m_drive.arcadeDrive(0.7, 0);   //0.7 is the speed (-1 to 1 scale), 0 is the rotation
       }

    @Override
    public boolean isFinished(){
        return (m_drive.getAverageEncoderDistance() >= (m_initialDistance + m_distance));
    }

    @Override
    public void end(boolean interrupted){
        onStart = true;
    }

}
