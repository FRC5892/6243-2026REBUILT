package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction for Shooter subsystem. Supports real hardware, simulation, and AdvantageKit
 * logging.
 */
public interface ShooterIO {

  class ShooterIOInputs implements org.littletonrobotics.junction.inputs.LoggableInputs {
    public double mainVelocityRPM;
    public double followerVelocityRPM;
    public double feederVelocityRPM;
    public double mainCurrentAmps;
    public double followerCurrentAmps;
    public double feederCurrentAmps;
    public double appliedOutput;
    public boolean isFiring;
    @Override
    public void toLog(org.littletonrobotics.junction.LogTable table) {
      table.put("MainVelocityRPM", mainVelocityRPM);
      table.put("FollowerVelocityRPM", followerVelocityRPM);
      table.put("FeederVelocityRPM", feederVelocityRPM);
      table.put("MainCurrentAmps", mainCurrentAmps);
      table.put("FollowerCurrentAmps", followerCurrentAmps);
      table.put("FeederCurrentAmps", feederCurrentAmps);
      table.put("AppliedOutput", appliedOutput);
      table.put("IsFiring", isFiring);
    }
    @Override
    public void fromLog(org.littletonrobotics.junction.LogTable table) {
      mainVelocityRPM = table.get("MainVelocityRPM", mainVelocityRPM);
      followerVelocityRPM = table.get("FollowerVelocityRPM", followerVelocityRPM);
      feederVelocityRPM = table.get("FeederVelocityRPM", feederVelocityRPM);
      mainCurrentAmps = table.get("MainCurrentAmps", mainCurrentAmps);
      followerCurrentAmps = table.get("FollowerCurrentAmps", followerCurrentAmps);
      feederCurrentAmps = table.get("FeederCurrentAmps", feederCurrentAmps);
      appliedOutput = table.get("AppliedOutput", appliedOutput);
      isFiring = table.get("IsFiring", isFiring);
    }
  }

  /** Update sensor readings */
  void updateInputs(ShooterIOInputs inputs);

  /** Closed-loop shooter velocity */
  void setShooterVelocity(double velocityRPM, double ffVolts);

  /** Open-loop voltage control for shooter */
  void setShooterVoltage(double volts);

  /** Open-loop voltage control for feeder */
  void setFeederVoltage(double volts);
}
