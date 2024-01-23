package frc.robot.utils;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PidControlledMotor {

  private RelativeEncoder _encoder;
  private AbsoluteEncoder _absEncoder;
  private CANSparkMax _motor;
  private SparkPIDController _pidController;
  private final int DEVICE_ID;

  public PidControlledMotor(int deviceId, MotorType mt) {
    DEVICE_ID = deviceId;
    _motor = new CANSparkMax(deviceId, (MotorType) mt);
    if (mt.equals(MotorType.kBrushless)) {
      this._motor.restoreFactoryDefaults();
      this._pidController = _motor.getPIDController();
    }
    this._encoder = _motor.getEncoder();
  }

  public double getPosition() {
    if (this._absEncoder != null) {
      return this._absEncoder.getPosition();
    } else {
      return this._encoder.getPosition();
    }
  }

  public void setConvFactor(double factor) {
    this._encoder.setPositionConversionFactor(factor);
  }

  public void setTargetSpeed(double speed) {
    this._pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void setVelocityFactor(double factor) {
    this._encoder.setVelocityConversionFactor(factor);
  }

  public void setAbsEncoder(AbsoluteEncoder enc) {
    this._absEncoder = enc;
    this._pidController.setFeedbackDevice(enc);
  }

  public void setEncoder(RelativeEncoder enc) {
    this._pidController.setFeedbackDevice(enc);
  }

  public void setPidConstants(PIDConstants constants) {
    this._pidController.setP(constants.kP);
    this._pidController.setI(constants.kI);
    this._pidController.setD(constants.kD);

  }

  public CANSparkMax getController() {
    return this._motor;
  }

  public RelativeEncoder get_encoder() {
    return this._encoder;
  }

  public CANSparkMax get_motor() {
    return this._motor;
  }

  public void setInverted(boolean inverted) {
    this._motor.setInverted(inverted);
  }

  public void setDrivePercent(double drivePercent) {

    this._motor.set(drivePercent);
    // this._pidController.setReference(drivePercent*Constants.SmartMotionConstants.MAX_VEL,CANSparkMax.ControlType.kVelocity);
  }

  public void setSetPoint(double setPoint) {
    this._encoder.setPosition(0);// resets the encoder pos (i think)
    this._pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public SparkMaxPIDController get_pidController() {
    return this._pidController;
  }

  public void setBreakMode() {
    this._motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoastMode() {
    this._motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

}
