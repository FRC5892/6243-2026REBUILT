package frc.robot.subsystems.shooter;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;

public class ShooterIOSparkMax implements ShooterIO {

    private final SparkMax shooterMotor;
    private final SparkMax followerMotor;
    private final SparkMax feederMotor;

    private final SparkClosedLoopController shooterPID;

    public ShooterIOSparkMax() {

        shooterMotor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(37, SparkLowLevel.MotorType.kBrushless);
        feederMotor   = new SparkMax(58, SparkLowLevel.MotorType.kBrushless);

        /* Shooter config */
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pidf(
                ShooterConstants.kP.get(),
                ShooterConstants.kI.get(),
                ShooterConstants.kD.get(),
                ShooterConstants.kFF.get()
        );
        shooterConfig.smartCurrentLimit(40);        // SPEED LIMIT
        shooterConfig.voltageCompensation(11.0);    // SPEED LIMIT
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.velocityConversionFactor(0.5); // SPEED LIMIT
        encoderConfig.positionConversionFactor(0.5);
        shooterConfig.apply(encoderConfig);

        shooterMotor.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        shooterPID = shooterMotor.getClosedLoopController();

        /* Follower config */
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(shooterMotor, true);
        followerConfig.smartCurrentLimit(40);        // SPEED LIMIT
        followerConfig.voltageCompensation(12.0);    // SPEED LIMIT
        followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        followerMotor.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        /* Feeder config */
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);          // SPEED LIMIT
        feederConfig.voltageCompensation(12.0);      // SPEED LIMIT
        feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.mainVelocityRPM = shooterMotor.getEncoder().getVelocity();
        inputs.followerVelocityRPM = followerMotor.getEncoder().getVelocity();
        inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();

        inputs.mainCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.followerCurrentAmps = followerMotor.getOutputCurrent();
        inputs.feederCurrentAmps = feederMotor.getOutputCurrent();

        inputs.appliedOutput = shooterMotor.getAppliedOutput();
        inputs.isFiring = feederMotor.getAppliedOutput() > 0.0;
    }

    @Override
    public void setShooterVelocity(double velocityRPM, double ffVolts) {
        shooterPID.setReference(velocityRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederMotor.setVoltage(volts);
    }
}
