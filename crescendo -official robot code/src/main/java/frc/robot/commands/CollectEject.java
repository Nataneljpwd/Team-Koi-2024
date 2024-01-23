package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class CollectEject extends Command {

  private final CollectionSubsystem collectionSubsystem = CollectionSubsystem.getInstance();
  private final TransferSubsystem transferSubsystem = TransferSubsystem.getInstance();
  private BooleanSupplier limitSwitch;
  private boolean collect;
  private Timer timer;

  public CollectEject(BooleanSupplier limitSwitch, boolean collect) {
    this.limitSwitch = limitSwitch;// the limit switch for the check when to stop the command
    this.collect = collect;
    addRequirements(collectionSubsystem, transferSubsystem);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collect) {
      collectionSubsystem.collect();
      transferSubsystem.transfer();
    } else {
      transferSubsystem.eject();
      collectionSubsystem.eject();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collect ? limitSwitch.getAsBoolean() : !limitSwitch.getAsBoolean() && timer.hasElapsed(1.5);
  }
}
