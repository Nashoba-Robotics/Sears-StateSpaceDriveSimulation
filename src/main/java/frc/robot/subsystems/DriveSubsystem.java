// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;



public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX leftTalon1 = new WPI_TalonSRX(1);
  private WPI_TalonSRX leftTalon2 = new WPI_TalonSRX(2);
  private WPI_TalonSRX leftTalon3 = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightTalon1 = new WPI_TalonSRX(4);
  private WPI_TalonSRX rightTalon2 = new WPI_TalonSRX(5);
  private WPI_TalonSRX rightTalon3 = new WPI_TalonSRX(6);

 // The motors on the left side of the drive.
 private final SpeedControllerGroup m_leftMotors =
 new SpeedControllerGroup(leftTalon1,leftTalon2,leftTalon3);

// The motors on the right side of the drive.
private final SpeedControllerGroup m_rightMotors =
new SpeedControllerGroup(rightTalon1,rightTalon2,rightTalon3);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // The left-side drive encoder
//  private final Encoder m_leftEncoder =
//       new Encoder(
//           DriveConstants.kLeftEncoderPorts[0],
//           DriveConstants.kLeftEncoderPorts[1],
//           DriveConstants.kLeftEncoderReversed);

//  // The right-side drive encoder
//   private final Encoder m_rightEncoder =
//       new Encoder(
//           DriveConstants.kRightEncoderPorts[0],
//           DriveConstants.kRightEncoderPorts[1],
//           DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
 // private EncoderSim m_leftEncoderSim;
  // private EncoderSim m_rightEncoderSim;
  private TalonSRXSimCollection leftTalonSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private ADXRS450_GyroSim m_gyroSim;

  final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  final double kGearRatio = 8.33; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
  final double kWheelRadiusInches = 3;
  final int k100msPerSecond = 10;

  TalonSRXSimCollection m_leftMotorSim = leftTalon1.getSimCollection();
  TalonSRXSimCollection m_rightMotorSim = rightTalon1.getSimCollection();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  //  m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    leftTalon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightTalon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
    //  m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    //  m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
                      nativeUnitsToDistanceMeters(leftTalon1.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(rightTalon1.getSelectedSensorPosition()));
    m_fieldSim.setRobotPose(getPose());
  }

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        leftTalon1.getMotorOutputVoltage(),
        rightTalon1.getMotorOutputVoltage());
    m_drivetrainSimulator.update(0.020);

  //  m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
  //  m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
  //  m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
  //  m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    /*m_leftMotorSim.setAnalogPosition((int)m_drivetrainSimulator.getLeftPositionMeters());
    m_leftMotorSim.setAnalogVelocity((int)m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightMotorSim.setAnalogPosition((int)m_drivetrainSimulator.getRightPositionMeters());
    m_rightMotorSim.setAnalogVelocity((int)m_drivetrainSimulator.getRightVelocityMetersPerSecond()); */

    m_leftMotorSim.setQuadratureRawPosition(
      distanceToNativeUnits(
        m_drivetrainSimulator.getLeftPositionMeters()));
m_leftMotorSim.setQuadratureVelocity(
      velocityToNativeUnits(
        m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
m_rightMotorSim.setQuadratureRawPosition(
      distanceToNativeUnits(
        m_drivetrainSimulator.getRightPositionMeters()));
m_rightMotorSim.setQuadratureVelocity(
      velocityToNativeUnits(
        m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
 
    m_leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //  return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  return new DifferentialDriveWheelSpeeds(leftTalon1.getSelectedSensorVelocity(), rightTalon1.getSelectedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
  //  m_leftEncoder.reset();
  //  m_rightEncoder.reset();
   // leftTalon1.
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftTalon1.getSelectedSensorPosition() + rightTalon1.getSelectedSensorPosition()) / 2.0;
  }

  // /**
  //  * Gets the left drive encoder.
  //  *
  //  * @return the left drive encoder
  //  */
  // //public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  // /**
  //  * Gets the right drive encoder.
  // //  *
  // //  * @return the right drive encoder
  // //  */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
