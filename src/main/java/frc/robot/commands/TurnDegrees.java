// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnDegrees extends Command {
  private final DriveSubsystem drive;
  private final double m_degrees;

  PIDController controller = new PIDController(0, 0, 0);

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired
   * rotation (in
   * degrees) and rotational speed.
   *
   * @param speed   The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive   The drive subsystem on which this command will run
   */
  public TurnDegrees(double degrees, DriveSubsystem drive) {
    //controller.enableContinuousInput(-180, 180);
    m_degrees = degrees;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    drive.arcadeDrive(0, 0);
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(0, controller.calculate(drive.getGyroAngleDegrees(), m_degrees));
    System.out.println(drive.getGyroAngleDegrees()+"   s");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
