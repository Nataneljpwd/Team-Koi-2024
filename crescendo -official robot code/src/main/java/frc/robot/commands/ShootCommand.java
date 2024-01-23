package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootCommand extends Command {

  private final ShootingSubsystem shootingSubsystem = ShootingSubsystem.getInstance();
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private Pose2d targerPose;

  public ShootCommand() {
    targerPose = DriverStation.Alliance.Blue == DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ? Constants.AutoConsts.BLUE_SPEAKER
        : Constants.AutoConsts.RED_SPEAKER;
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the position of the robot relatively to the speaker
    Pose2d botPose = swerveSubsystem.getPose();// TODO:init robot rotation from april tags on enable (auto init)
    // TODO: add the calculation and set pid constants and when we are aimed, we
    // shoot and the we stop

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
