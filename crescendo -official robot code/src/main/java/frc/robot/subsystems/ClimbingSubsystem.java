package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PidControlledMotorGroup;

public class ClimbingSubsystem extends SubsystemBase {

  private static ClimbingSubsystem instance;

  private PidControlledMotorGroup climbingMotors;
  private DigitalInput limitTop, limitBottom;

  private ClimbingSubsystem() {
    climbingMotors = new PidControlledMotorGroup(new int[] { 8, 9 }, MotorType.kBrushed,
        new PIDConstants[] { new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0) });
    climbingMotors.setOneInverted(true);
    limitTop = new DigitalInput(0);
    limitBottom = new DigitalInput(1);
  }

  public static ClimbingSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbingSubsystem();
    }
    return instance;
  }

  public void extend() {
    if (!limitTop.get()) {
      climbingMotors.setDrivePercent(0.5);
    }
  }

  public void retract() {
    if (!limitBottom.get()) {
      climbingMotors.setDrivePercent(-0.5);
    }
  }
}
