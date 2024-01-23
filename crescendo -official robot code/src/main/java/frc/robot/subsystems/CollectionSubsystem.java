package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PidControlledMotor;
import frc.robot.utils.PidControlledMotorGroup;

public class CollectionSubsystem extends SubsystemBase {

  private static CollectionSubsystem instance;

  private PidControlledMotorGroup collectionMotors;

  private final double COLLECTION_SPEED = 0.7;

  public static CollectionSubsystem getInstance() {
    if (instance == null) {
      instance = new CollectionSubsystem();
    }
    return instance;
  }

  private CollectionSubsystem() {
    collectionMotors = new PidControlledMotorGroup(new int[] { 4, 5 }, MotorType.kBrushless,
        new PIDConstants[] { new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0) });
  }

  /**
   * @param dr the percent of the force the motors will work at
   */
  private void setCollectionPercent(double dr) {
    collectionMotors.setDrivePercent(dr);
  }

  public void collect() {
    setCollectionPercent(COLLECTION_SPEED);
  }

  public void eject() {
    setCollectionPercent(-COLLECTION_SPEED);
  }

}
