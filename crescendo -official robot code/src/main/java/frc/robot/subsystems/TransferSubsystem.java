package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PidControlledMotorGroup;

public class TransferSubsystem extends SubsystemBase {

  private static TransferSubsystem instance;

  private final double TRANSFER_SPEED = 0.7;

  private PidControlledMotorGroup transferMotors;

  public static TransferSubsystem getInstance() {
    if (instance == null) {
      instance = new TransferSubsystem();
    }
    throw new RuntimeException("wont run");
  }

  private TransferSubsystem() {
    transferMotors = new PidControlledMotorGroup(new int[] { 6, 7 }, MotorType.kBrushless,
        new PIDConstants[] { new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0) });
    transferMotors.setOneInverted(true);
  }

  private void setTransferPercent(double dr) {
    transferMotors.setDrivePercent(dr);
  }

  public void transfer() {
    setTransferPercent(TRANSFER_SPEED);
  }

  public void eject() {
    setTransferPercent(-TRANSFER_SPEED);
  }
}
