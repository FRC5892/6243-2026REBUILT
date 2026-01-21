package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {

  private double armPos = 0.0;
  private double armVel = 0.0;
  private double armVolts = 0.0;

  private double rollerVel = 0.0;
  private double rollerVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    armVel += armVolts * 0.02;
    armPos += armVel * 0.02;

    rollerVel = rollerVolts * 100.0;

    inputs.armPositionRad = armPos;
    inputs.armVelocityRadPerSec = armVel;
    inputs.armAppliedVolts = armVolts;
    inputs.armCurrentAmps = Math.abs(armVolts) * 2.0;

    inputs.rollerVelocityRadPerSec = rollerVel;
    inputs.rollerAppliedVolts = rollerVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerVolts) * 1.5;
  }

  @Override
  public void setArmVoltage(double volts) {
    armVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerVolts = volts;
  }
}
