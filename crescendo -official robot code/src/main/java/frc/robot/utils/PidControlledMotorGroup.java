package frc.robot.utils;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class PidControlledMotorGroup {
  private PidControlledMotor motor1, motor2;

  public PidControlledMotorGroup(int[] motorIds, MotorType mt, PIDConstants[] constants) {
    motor1 = new PidControlledMotor(motorIds[0], mt);
    motor2 = new PidControlledMotor(motorIds[1], mt);
    motor1.setPidConstants(constants[0]);
    motor2.setPidConstants(constants[1]);
    motor2.get_motor().follow(motor1.get_motor());
  }

  public MotorController[] getControllers() {
    return new MotorController[] { motor1.getController(), motor2.getController() };
  }

  public void setVelocityFactor(double factor) {
    motor1.setVelocityFactor(factor);
    motor2.setVelocityFactor(factor);
  }

  public void setTargetSpeed(double rpm) {
    motor1.setTargetSpeed(rpm);
    motor2.setTargetSpeed(rpm);
  }

  public void setDrivePercent(double percent) {
    motor1.setDrivePercent(percent);
  }

  public void setInverted(boolean inverted) {
    motor1.setInverted(inverted);
    motor2.setInverted(inverted);
  }

  public void setOneInverted(boolean inverted) {
    motor2.setInverted(inverted);
  }

  public double getPosition() {
    return Math.abs(this.motor1.get_encoder().getPosition() / 2.0)
        + Math.abs(this.motor2.get_encoder().getPosition() / 2.0);
  }

  public void reset() {
    this.motor1.get_encoder().setPosition(0);
    this.motor2.get_encoder().setPosition(0);
  }

  public void setSetPoint(double sp) {
    motor1.setSetPoint(sp);
  }

  public void setConvFactor(double factor) {
    motor1.setConvFactor(factor);
    motor2.setConvFactor(factor);
  }

  public double getVelocity() {
    return this.motor2.get_encoder().getVelocity() / 2
        + this.motor1.get_encoder().getVelocity() / 2;
  }

  public void setBreakMode() {
    this.motor1.setBreakMode();
    this.motor2.setBreakMode();
  }

  public void setCoastMode() {
    this.motor2.setCoastMode();
    this.motor1.setCoastMode();
  }
}
