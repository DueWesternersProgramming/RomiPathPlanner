// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class TankDrive extends Command {
  private final DriveSubsystem m_drivetrain;
  private final Supplier<Double> leftSupplier;
  private final Supplier<Double> rightSupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain          The drivetrain subsystem on which this command
   *                            will run
   * @param xaxisSpeedSupplier  Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public TankDrive(
      DriveSubsystem drivetrain,
      Supplier<Double> leftSupplier,
      Supplier<Double> rightSupplier) {
    m_drivetrain = drivetrain;
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.tankDrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(leftSupplier.get(), rightSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
