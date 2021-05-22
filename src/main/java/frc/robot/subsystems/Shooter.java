package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Util;

public class Shooter extends SubsystemBase {


    public enum ShooterControlMode {
        MANUAL, TRACK_LIMELIGHT
    };

    public enum HoodControlMode {
        MOTION_MAGIC, VELOCITY, MANUAL, MOTION_MAGIC_TRACK_LIMELIGHT
    };

    // Conversions
    private final double KICKER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double INTAKE_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double HOOD_OUTPUT_TO_ENCODER_RATIO = 4.0 * 322.0 / 20.0; // Added 4:1 VP
    private final double HOOD_REVOLUTIONS_TO_ENCODER_TICKS = HOOD_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private final double HOOD_DEGREES_TO_ENCODER_TICKS = HOOD_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;

    // Motion Magic
    private static final int kHoodMotionMagicSlot = 0;
    private HoodControlMode hoodControlMode = HoodControlMode.MANUAL;
    private ShooterControlMode shooterControlMode = ShooterControlMode.MANUAL;

    // Motor Controllers
    private TalonFX shooterMainMaster;
    private TalonFX shooterMainSlave;
    private TalonFX shooterKicker;
    private TalonFX shooterIntake;
    private TalonFX shooterHood;

    // Misc
    private double homePositionAngleDegrees = Constants.HOOD_COMPETITION_HOME_POSITION_DEGREES;
    private double targetPositionTicks = 0;
    private boolean isReady;
    private double lastRPM;
    private double lastHoodAngle;

    private final static Shooter INSTANCE = new Shooter();

    private Shooter() {
        shooterMainMaster = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_MASTER_CAN_ID);
        shooterMainSlave = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID);
        shooterKicker = new TalonFX(Constants.SHOOTER_KICKER_MOTOR_CAN_ID);
        shooterIntake = new TalonFX(Constants.SHOOTER_INTAKE_MOTOR_CAN_ID);
        shooterHood = new TalonFX(Constants.SHOOTER_HOOD_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        shooterMainMaster.configAllSettings(configs);
        shooterMainSlave.configAllSettings(configs);
        shooterKicker.configAllSettings(configs);
        shooterIntake.configAllSettings(configs);
        shooterHood.configAllSettings(configs);

        shooterMainMaster.setInverted(TalonFXInvertType.CounterClockwise);
        shooterMainMaster.setNeutralMode(NeutralMode.Coast);

        shooterMainSlave.setInverted(TalonFXInvertType.Clockwise);
        shooterMainSlave.setNeutralMode(NeutralMode.Coast);
        shooterMainSlave.follow(shooterMainMaster);

        shooterKicker.setInverted(TalonFXInvertType.Clockwise);
        shooterKicker.setNeutralMode(NeutralMode.Coast);

        shooterIntake.setInverted(TalonFXInvertType.CounterClockwise);
        shooterIntake.setNeutralMode(NeutralMode.Coast);

        shooterHood.setInverted(TalonFXInvertType.Clockwise);
        shooterHood.setNeutralMode(NeutralMode.Brake);
        shooterHood.configMotionCruiseVelocity(3000);
        shooterHood.configMotionAcceleration(6000);
 //       shooterHood.configMotionSCurveStrength(4);

        StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 60;
        statorCurrentConfigs.enable = true;
        statorCurrentConfigs.triggerThresholdCurrent = 80;
        statorCurrentConfigs.triggerThresholdTime = 0.5;

        shooterMainMaster.configStatorCurrentLimit(statorCurrentConfigs);
        shooterMainSlave.configStatorCurrentLimit(statorCurrentConfigs);
        shooterKicker.configStatorCurrentLimit(statorCurrentConfigs);
        shooterIntake.configStatorCurrentLimit(statorCurrentConfigs);

        StatorCurrentLimitConfiguration statorCurrentHoodConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentHoodConfigs.currentLimit = 30;
        statorCurrentHoodConfigs.enable = true;
        statorCurrentHoodConfigs.triggerThresholdCurrent = 60;
        statorCurrentHoodConfigs.triggerThresholdTime = 0.5;

        shooterHood.configStatorCurrentLimit(statorCurrentHoodConfigs);

        shooterMainMaster.config_kF(0, 0.05);
        shooterMainMaster.config_kP(0, 0.5);
        shooterMainMaster.config_kI(0, 0.0);
        shooterMainMaster.config_kD(0, 0.0);

        shooterKicker.config_kF(0, 0.045);
        shooterKicker.config_kP(0, 0.2);
        shooterKicker.config_kI(0, 0.00);
        shooterKicker.config_kD(0, 0.0);  // 0.6

        shooterIntake.config_kF(0, 0.05);
        shooterIntake.config_kP(0, 0.1);
        shooterIntake.config_kI(0, 0.0);
        shooterIntake.config_kD(0, 0.0);  // 0.6

        shooterHood.config_kF(kHoodMotionMagicSlot, 0.045);
        shooterHood.config_kP(kHoodMotionMagicSlot, 1.0);//.9
        shooterHood.config_kI(kHoodMotionMagicSlot, 0.008);//.008
        shooterHood.config_kD(kHoodMotionMagicSlot, 0.0);
        shooterHood.config_IntegralZone(kHoodMotionMagicSlot, (int)(5.0 * HOOD_DEGREES_TO_ENCODER_TICKS));
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public void setMainSpeed(double speed) {
        shooterMainMaster.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Shooter Speed");
    }

    public void resetShooterPosition() {
        shooterMainMaster.setSelectedSensorPosition(0);
    }

    public double getMainRotations() {
        return shooterMainMaster.getSelectedSensorPosition() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    }

    public double getMainRPM() {
        return shooterMainMaster.getSelectedSensorVelocity() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void setMainRPM(double rpm) {
        setShooterControlMode(ShooterControlMode.MANUAL);
        shooterMainMaster.set(ControlMode.Velocity, ShooterRPMToNativeUnits(rpm));
    }

    public double ShooterRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;
    }

    public void setKickerSpeed(double speed) {
        shooterKicker.set(ControlMode.PercentOutput, speed);
    }

    public void resetKickerPosition() {
        shooterKicker.setSelectedSensorPosition(0);
    }

    public double getKickerRotations() {
        return shooterKicker.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / KICKER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getKickerRPM() {
        return shooterKicker.getSelectedSensorVelocity() / KICKER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void setKickerRPM(double rpm) {
        setShooterControlMode(ShooterControlMode.MANUAL);
        shooterKicker.set(ControlMode.Velocity, KickerRPMToNativeUnits(rpm));
    }

    public double KickerRPMToNativeUnits(double rpm) {
        return rpm * KICKER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;
    }

    public void setIntakeSpeed(double speed) {
        shooterIntake.set(ControlMode.PercentOutput, speed);
    }

    public void resetIntakePosition() {
        shooterIntake.setSelectedSensorPosition(0);
    }

    public double getIntakeRotations() {
        return shooterIntake.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INTAKE_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIntakeRPM() {
        return shooterIntake.getSelectedSensorVelocity() / INTAKE_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void setIntakeRPM(double rpm) {
        shooterIntake.set(ControlMode.Velocity, KickerRPMToNativeUnits(rpm));
    }

    public double IntakeRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;

    }

    // Turret Control Mode
    public synchronized void setShooterControlMode(ShooterControlMode controlMode) {
        this.shooterControlMode = controlMode;
    }

    public synchronized ShooterControlMode getShooterControlMode() {
        return this.shooterControlMode;
    }

    private synchronized void setHoodControlMode(HoodControlMode controlMode) {
        this.hoodControlMode = controlMode;
    }

    private synchronized Shooter.HoodControlMode getHoodControlMode() {
        return this.hoodControlMode;
    }

    public void setHoodSpeed(double speed) {
        shooterHood.set(ControlMode.PercentOutput, speed);
    }

    public void resetHoodHomePosition() {
        shooterHood.setSelectedSensorPosition(0);
    }

    public double getHoodAngleAbsoluteDegrees() {
        return (double)shooterHood.getSelectedSensorPosition() / HOOD_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }

    public double hoodRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;

    }

    // Motion Magic
    public synchronized void setHoodMotionMagicPositionAbsolute(double angle) {
        if (getHoodControlMode() != HoodControlMode.MOTION_MAGIC) {
            setHoodControlMode(HoodControlMode.MOTION_MAGIC);
        }
        setHoodMotionMagicPositionAbsoluteInternal(angle);
    }

    public synchronized void setHoodMotionMagicPositionAbsoluteInternal(double angle) {
        shooterHood.selectProfileSlot(kHoodMotionMagicSlot, 0);
        double limitedAngle = limitHoodAngle(angle);
        targetPositionTicks = getHoodEncoderTicksAbsolute(limitedAngle);
        shooterHood.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.07);
    }

    public synchronized boolean hasFinishedHoodTrajectory() {
        return hoodControlMode == HoodControlMode.MOTION_MAGIC
                && Util.epsilonEquals(shooterHood.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    private double getHoodEncoderTicksAbsolute(double angle) {
        double positionDegrees = angle - homePositionAngleDegrees;
        return (int) (positionDegrees * HOOD_DEGREES_TO_ENCODER_TICKS);
    }

    public synchronized double getHoodSetpointAngle() {
        return hoodControlMode == HoodControlMode.MOTION_MAGIC
                ? targetPositionTicks / HOOD_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees
                : Double.NaN;
    }

    private double limitHoodAngle(double targetAngle) {
        if (targetAngle < Constants.HOOD_MIN_ANGLE_DEGREES) {
            return Constants.HOOD_MIN_ANGLE_DEGREES;
        } else if (targetAngle > Constants.HOOD_MAX_ANGLE_DEGREES) {
            return Constants.HOOD_MAX_ANGLE_DEGREES;
        }

        return targetAngle;
    }

    // Shoot while driving
    public double getRPMFromDistance() {
        return 7.98248 * Limelight.getInstance().getDistanceFromTargetInches() + 1542.75;
    }

    public double getHoodAngleFromDistance() {
        return 0.0809014 * Limelight.getInstance().getDistanceFromTargetInches() + 27.8;
    }

    public void setShooterLimelightTrackMode() {
        setShooterControlMode(ShooterControlMode.TRACK_LIMELIGHT);
        updateLimelightTrackV2();
    }

    public void setReady(boolean isReady) {
        this.isReady = isReady;
    }

    public boolean isReady() {
        return isReady;
    }

    private void updateLimelightTrackV2() {
        double rpm = lastRPM;
        double hoodAngle = lastHoodAngle;
        if (Limelight.getInstance().isOnTarget()) {            
            rpm = getRPMFromDistance();
            hoodAngle = getHoodAngleFromDistance();
            lastRPM = rpm;
            lastHoodAngle = hoodAngle;
        } 
        shooterMainMaster.set(ControlMode.Velocity, ShooterRPMToNativeUnits(rpm));
        shooterKicker.set(ControlMode.Velocity, IntakeRPMToNativeUnits(rpm));
        setHoodMotionMagicPositionAbsolute(hoodAngle);
    }

    public void periodic() {
        if (getShooterControlMode() == ShooterControlMode.TRACK_LIMELIGHT) {
            updateLimelightTrackV2();
        }
 //       SmartDashboard.putNumber("Shooters Rotations", getMainRotations());
        SmartDashboard.putNumber("Shooters RPM", getMainRPM());
//        SmartDashboard.putNumber("Shooters RPM Graph", getShooterRPM());
//        SmartDashboard.putNumber("Shooters Velocity Native", shooterMainMaster.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Shooters input", shooterMainMaster.getMotorOutputPercent());
//        SmartDashboard.putNumber("Shooters Stator Current", shooterMainMaster.getStatorCurrent());
//        SmartDashboard.putNumber("Shooters Supply Current", shooterMainMaster.getSupplyCurrent());
//        SmartDashboard.putNumber("Shooters2 Supply Current", shooterMainSlave.getSupplyCurrent());
 //       SmartDashboard.putNumber("Kicker Rotations", getKickerRotations());
 //       SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
//        SmartDashboard.putNumber("Kicker RPM Graph", getKickerRPM());
//        SmartDashboard.putNumber("Kicker Velocity Native", shooterKicker.getSelectedSensorVelocity());
//       SmartDashboard.putNumber("Kicker Stator Current", shooterKicker.getStatorCurrent());
//        SmartDashboard.putNumber("Kicker Supply Current", shooterKicker.getSupplyCurrent());
//        SmartDashboard.putNumber("Intake Rotations", getIntakeRotations());
 //       SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
//        SmartDashboard.putNumber("Intake RPM Graph", getIntakeRPM());
//        SmartDashboard.putNumber("Intake Velocity Native", shooterIntake.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Intake Stator Current", shooterIntake.getStatorCurrent());
//        SmartDashboard.putNumber("Intake Supply Current", shooterIntake.getSupplyCurrent());
        SmartDashboard.putNumber("Hood Angle", getHoodAngleAbsoluteDegrees());
//        SmartDashboard.putNumber("Hood Velocity", shooterHood.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Hood Position", shooterHood.getSelectedSensorPosition());
        SmartDashboard.putBoolean("Shooter Ready", isReady);
        SmartDashboard.putNumber("RPM From Distance", getRPMFromDistance());
        SmartDashboard.putNumber("Hood Angle From Distance", getHoodAngleFromDistance());

    }
}

