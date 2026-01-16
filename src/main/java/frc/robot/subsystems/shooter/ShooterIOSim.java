package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {

    private double velocity = 0.0;
    private boolean firing = false;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.mainVelocityRPM = velocity;
        inputs.followerVelocityRPM = velocity;
        inputs.feederVelocityRPM = firing ? 100 : 0;

        inputs.mainCurrentAmps = 5.0;
        inputs.followerCurrentAmps = 5.0;
        inputs.feederCurrentAmps = firing ? 2.0 : 0.5;

        inputs.appliedOutput = velocity / 5000.0;
        inputs.isFiring = firing;
    }

    @Override
    public void setShooterVelocity(double velocityRPM, double ffVolts) {
        this.velocity = velocityRPM;
    }

    @Override
    public void setShooterVoltage(double volts) {
        this.velocity = volts * 500.0;
    }

    @Override
    public void setFeederVoltage(double volts) {
        this.firing = volts > 0.0;
    }
}
