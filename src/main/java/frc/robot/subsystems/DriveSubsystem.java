// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class DriveSubsystem extends SubsystemBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Drivetrain.kTrackWidthMeters);

  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Drivetrain.kWheelDiameterInch) / Drivetrain.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Drivetrain.kWheelDiameterInch) / Drivetrain.kCountsPerRevolution);
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(getGyroAngleDegrees())),
        Units.inchesToMeters(m_leftEncoder.getDistance()), Units.inchesToMeters(m_rightEncoder.getDistance()));
        

    AutoBuilder.configureRamsete(this::getPose, this::resetOdometry, this::getChassisSpeeds, this::driveChassisSpeeds,0,1,
        new ReplanningConfig(false, true), () -> false, this);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDriveSpeedsMetersPerSec(double x, double rot) {
    // Convert from meters per second to percentage of max speed (-1.0 to 1.0)
    double xPercent = x / Drivetrain.kMaxSpeedMetersPerSecond;
    double rotPercent = rot / Drivetrain.kMaxSpeedMetersPerSecond;

    // Ensure the values stay within the range of -1.0 to 1.0
    xPercent = Math.max(Math.min(xPercent, 1.0), -1.0);
    rotPercent = Math.max(Math.min(rotPercent, 1.0), -1.0);

    // Use tankDrive to apply the converted percentages
    m_diffDrive.arcadeDrive(xPercent, rotPercent);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDriveSpeedsMetersPerSec(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());

  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    m_odometry.resetPosition(new Rotation2d(Math.toRadians(getGyroAngleDegrees())), Units.inchesToMeters(m_leftEncoder.getDistance()), Units.inchesToMeters(m_rightEncoder.getDistance()), pose);
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getGyroAngleDegrees() {
    return -m_gyro.getAngle();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  private void updateOdometry(){
    m_odometry.update(new Rotation2d(Math.toRadians(getGyroAngleDegrees())), Units.inchesToMeters(m_leftEncoder.getDistance()), Units.inchesToMeters(m_rightEncoder.getDistance()));
    m_field2d.setRobotPose(getPose());
  }

  @Override
  public void periodic() {
    updateOdometry();
    
    SmartDashboard.putData("Pose2d", m_field2d);

    // This method will be called once per scheduler run
  }

}
