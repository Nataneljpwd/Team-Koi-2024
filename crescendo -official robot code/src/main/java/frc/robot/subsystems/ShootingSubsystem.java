package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PidControlledMotor;
import frc.robot.utils.PidControlledMotorGroup;

public class ShootingSubsystem extends SubsystemBase {

  private static ShootingSubsystem instance;

  private PidControlledMotorGroup shootingMotors;
  private PidControlledMotor angleMotor;
  private PidControlledMotor feedMotor;

  private ShootingSubsystem() {
    shootingMotors = new PidControlledMotorGroup(new int[] { 0, 1 }, MotorType.kBrushless,
        new PIDConstants[] { new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0) });
    shootingMotors.setVelocityFactor(1);
    angleMotor = new PidControlledMotor(2, MotorType.kBrushless);
    angleMotor.setPidConstants(new PIDConstants(0, 0, 0));
    angleMotor.setConvFactor(360);
    feedMotor = new PidControlledMotor(3, MotorType.kBrushless);
    feedMotor.setPidConstants(new PIDConstants(0, 0, 0));
    feedMotor.setVelocityFactor(1);
  }

  public static ShootingSubsystem getInstance() {
    if (instance == null) {
      instance = new ShootingSubsystem();
    }
    return instance;
  }

  /**
   * @param mps the note speed in meters per second for the rmp
   * @return the rpm needed for that speed
   */
  private double getRPMForNodeExitSpeed(double mps) {
    return mps * 1;// TODO: find the funciton
  }

  public void spinMotorsToGetTargetSpeed(double mps) {
    shootingMotors.setTargetSpeed(getRPMForNodeExitSpeed(mps));
  }

  /**
   * @param angle the target angle in deg
   */
  public void setAngle(double angle) {
    angleMotor.setSetPoint(angle);
  }

  public double getShooterAngle() {
    return angleMotor.getPosition();
  }

  /**
   * @param speed the target rpm
   */
  public void setShooterSpeed(double speed) {
    shootingMotors.setSetPoint(speed);
  }

  public void feed(double speed) {
    feedMotor.setTargetSpeed(speed);
  }

  // public Command getShootCommand(SwerveSubsystem swerveSubsystem) {
  // Command aimCommand = new InstantCommand(()-> {§});
  // Command shootCommand = new SequentialCommandGroup(
  // )
  // }

}
