package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Util;

public class Turret extends SubsystemBase {

    public static enum TurretControlMode {
        MOTION_MAGIC, MOTION_MAGIC_TRACK_GYRO, MOTION_MAGIC_TRACK_LIMELIGHT, POSITION, VELOCITY, MANUAL
    };

    // Conversions
    private static final double TURRET_OUTPUT_TO_ENCODER_RATIO = 4.0 * 168.0 / 28.0;  // 4.0 VP * 168T Ring gear / 28T Pinion
    public static final double TURRET_REVOLUTIONS_TO_ENCODER_TICKS = TURRET_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double TURRET_DEGREES_TO_ENCODER_TICKS = TURRET_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;

    // Motion Magic
    private static final int kTurretMotionMagicSlot = 0;
    private static final int kTurretPositionSlot = 1;
    private TurretControlMode turretControlMode = TurretControlMode.MANUAL;

    // Motor Controllers
    private final TalonFX turretMotor;

    // Sensors
    private DigitalInput minRevTurretSensor;
    private DigitalInput maxRevTurretSensor;
    // Misc
    private double homePositionAngleDegrees = Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES;
    private double targetPositionTicks = 0;
 
    private double gyroTrackOffsetAngle;
    private double limelightTrackOffsetAngle;
 
    public double rCurrPoseX;
    public double rCurrPoseY;
    public double rDistToGoal;
    public double rTurretGoalAngle;
    public double rTurretGoalOffsetAngle;
    public double offsetY;
    public double movingTurrOffsetAngle;
    public double turretGoalAngleLag;
    public double rVelocity;

    // Subsystem Instance
    private final static Turret INSTANCE = new Turret();

    // Constructor
    private Turret() {
        turretMotor = new TalonFX(Constants.TURRET_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        turretMotor.configAllSettings(configs);

        turretMotor.setInverted(TalonFXInvertType.CounterClockwise);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configMotionCruiseVelocity(6000);
        turretMotor.configMotionAcceleration(14000);
        turretMotor.configMotionSCurveStrength(4);

        minRevTurretSensor = new DigitalInput(Constants.TURRET_MIN_REV_SENSOR_DIO_ID);
        maxRevTurretSensor = new DigitalInput(Constants.TURRET_MAX_REV_SENSOR_DIO_ID);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 40;
        statorCurrentConfigs.enable = true;
        statorCurrentConfigs.triggerThresholdCurrent = 80;
        statorCurrentConfigs.triggerThresholdTime = 0.5;
        turretMotor.configStatorCurrentLimit(statorCurrentConfigs);

        turretMotor.config_kF(kTurretMotionMagicSlot, 0.0); //0.04
        turretMotor.config_kP(kTurretMotionMagicSlot, 0.9);
        turretMotor.config_kI(kTurretMotionMagicSlot, 0.004);
        turretMotor.config_kD(kTurretMotionMagicSlot, 0.0);
        turretMotor.config_IntegralZone(kTurretMotionMagicSlot, (int)(5.0 * TURRET_DEGREES_TO_ENCODER_TICKS));

        turretMotor.config_kF(kTurretPositionSlot, 0.0);  //0.03
        turretMotor.config_kP(kTurretPositionSlot, 0.3);
        turretMotor.config_kI(kTurretPositionSlot, 0.0002);
        turretMotor.config_kD(kTurretPositionSlot, 0.1);
        turretMotor.config_IntegralZone(kTurretPositionSlot, (int)(5.0 * TURRET_DEGREES_TO_ENCODER_TICKS));
    }

    public static Turret getInstance() {
        return INSTANCE;
    }

    // Turret Control Mode
    private synchronized void setTurretControlMode(TurretControlMode controlMode) {
        this.turretControlMode = controlMode;
    }

    private synchronized TurretControlMode getTurretControlMode() {
        return this.turretControlMode;
    }

    // Manual Control
    public void setTurretSpeed(final double speed) {
        setTurretControlMode(TurretControlMode.MANUAL);
        turretMotor.set(ControlMode.PercentOutput, speed);
    }

    // Velocity Control
    public double getTurretRPM() {
        return turretMotor.getSelectedSensorVelocity() / TURRET_REVOLUTIONS_TO_ENCODER_TICKS  * 10.0 * 60.0;
    }

    public void setTurretRPM(final double rpm) {
        setTurretControlMode(TurretControlMode.VELOCITY);
        turretMotor.set(ControlMode.Velocity, TurretRPMToNativeUnits(rpm));
    }

    // Motion Magic
     public synchronized void setTurretMotionMagicPositionAbsolute(double angle) {
        if (getTurretControlMode() != TurretControlMode.MOTION_MAGIC) {
            setTurretControlMode(TurretControlMode.MOTION_MAGIC);
        }
        setTurretMotionMagicPositionAbsoluteInternal(angle);
    }

    public synchronized void setTurretMotionMagicPositionAbsoluteInternal(double angle) {
        turretMotor.selectProfileSlot(kTurretMotionMagicSlot, 0);
        targetPositionTicks = getTurretEncoderTicksAbsolute(limitTurretAngle(angle));
 //       System.out.println("Set point MM absolute encoder ticks = " + targetPositionTicks);
        turretMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setTurretMotionMagicPositionRelative(double delta_angle) {
        if (getTurretControlMode() != TurretControlMode.MOTION_MAGIC) {
            setTurretControlMode(TurretControlMode.MOTION_MAGIC);
        }
        setTurretMotionMagicPositionRelativeInternal(delta_angle);
    }

    public synchronized void setTurretMotionMagicPositionRelativeInternal(double delta_angle) {
        turretMotor.selectProfileSlot(kTurretMotionMagicSlot, 0);
        targetPositionTicks = getTurretEncoderTicksRelative(delta_angle);
//        System.out.println("Set point MM relative encoder ticks = " + targetPositionTicks);
        turretMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setTurretPositionRelative(double delta_angle) {
        if (getTurretControlMode() != TurretControlMode.POSITION) {
            setTurretControlMode(TurretControlMode.POSITION);
        }
        turretMotor.selectProfileSlot(kTurretPositionSlot, 0);
        targetPositionTicks = getTurretEncoderTicksRelative(delta_angle);
        System.out.println("Set position ticks = " + targetPositionTicks);
        System.out.println("Set point position encoder ticks = " + targetPositionTicks);
        turretMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return turretControlMode == TurretControlMode.MOTION_MAGIC
                && Util.epsilonEquals(turretMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    public synchronized double getTurretSetpointAngle() {
        return turretControlMode == TurretControlMode.MOTION_MAGIC
                ? targetPositionTicks / TURRET_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees
                : Double.NaN;
    }

    // Reset methods
    public synchronized void resetHomePosition(double homePositionAngleDegrees) {
        turretMotor.setSelectedSensorPosition(0);
        this.homePositionAngleDegrees = homePositionAngleDegrees;
        System.out.println("Home angle = " + homePositionAngleDegrees);
    }

    // Getters and Converters
    public double getTurretAngleAbsoluteDegrees() {
        return (double)turretMotor.getSelectedSensorPosition() / TURRET_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }

    public double TurretRPMToNativeUnits(final double rpm) {
        return rpm * TURRET_REVOLUTIONS_TO_ENCODER_TICKS / 10.0 / 60.0;
    }

    private int getTurretEncoderTicksAbsolute(double angle) {
        double positionDegrees = angle - homePositionAngleDegrees;
        return (int) (positionDegrees * TURRET_DEGREES_TO_ENCODER_TICKS);
    }

    private int getTurretEncoderTicksRelative(double delta_angle) {
        double positionDegrees = limitTurretAngle(getTurretAngleAbsoluteDegrees() + delta_angle) - homePositionAngleDegrees;
        return (int) (positionDegrees * TURRET_DEGREES_TO_ENCODER_TICKS);
    }

    private double limitTurretAngle(double targetAngle) {
        if (targetAngle < Constants.TURRET_MIN_ANGLE_DEGREES) {
            return Constants.TURRET_MIN_ANGLE_DEGREES;
        } else if (targetAngle > Constants.TURRET_MAX_ANGLE_DEGREES) {
            return Constants.TURRET_MAX_ANGLE_DEGREES;
        }

        return targetAngle;
    }

    public boolean getMaxTurretSensor(){
        return !maxRevTurretSensor.get();
    }
    public boolean getMinTurretSensor(){
        return !minRevTurretSensor.get();
    }

    public void setGyroTrackMode(double gyroOffsetAngle) {
        this.gyroTrackOffsetAngle = gyroOffsetAngle;
        setTurretControlMode(TurretControlMode.MOTION_MAGIC_TRACK_GYRO);
        updateGyroTrack();
    }

    private void updateGyroTrack() {
        // double gyroMirror = Util.normalizeAngle90ToMinus270(Drive.getInstance().getGyroFusedHeadingAngleDeg());
        // if (Math.abs(gyroMirror) < 90) {
        //     gyroMirror = -gyroMirror;
        // }
        // else {
        //     gyroMirror = (-180 - gyroMirror) - 180;
        // }
        setTurretMotionMagicPositionAbsoluteInternal(Util.normalizeAngle90ToMinus270(-Drive.getInstance().getGyroFusedHeadingAngleDeg() + gyroTrackOffsetAngle));
    }
    
    public void setPositionToGoalAngle(Drive drive) {
        double angle = Util.normalizeAngle90ToMinus270(getTurretToGoalAngle(drive) - drive.getGyroFusedHeadingAngleDeg());
        setTurretMotionMagicPositionAbsolute(angle);
    }

    public void setLimelightTrackMode(double limelightTrackOffsetAngle) {
        this.limelightTrackOffsetAngle = limelightTrackOffsetAngle;
        setTurretControlMode(TurretControlMode.MOTION_MAGIC_TRACK_LIMELIGHT);
        updateLimelightTrack();
    }

    public void setLimelightFixedMode() {
        setTurretControlMode(TurretControlMode.MOTION_MAGIC);
        updateLimelightTrack();
    }

    private void updateLimelightTrack() {
        Limelight limelight = Limelight.getInstance();
 
        // If target is found use limelight to adjust turret angle.  If no target is found, 
        // stay at the same turret angle.  Drivers can turn robot until the target is acquired.
        if (limelight.isOnTarget()) {
            double deltaLimelightAngle = -limelight.getFilteredTx() + limelightTrackOffsetAngle;
            setTurretMotionMagicPositionRelativeInternal(deltaLimelightAngle);
        }
    }

    private double getTurretToGoalAngle(Drive drive) {
        rCurrPoseX = Units.metersToInches(drive.getPose().getTranslation().getX());
        rCurrPoseY = Units.metersToInches(drive.getPose().getTranslation().getY());
        rDistToGoal = Units.metersToInches(drive.getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN));
        rTurretGoalAngle = Math.toDegrees(Math.atan2(rCurrPoseY + 95, rCurrPoseX));
        double rTurretBackGoalAngle = Math.toDegrees(Math.atan2(rCurrPoseY + 95, rCurrPoseX + 32));
        rTurretGoalOffsetAngle = rTurretBackGoalAngle - rTurretGoalAngle;

//        offsetY = Math.abs(drive.getLeftMetersPerSecond()) * Constants.FLIGHT_TIME_OF_BALL;
//        movingTurrOffsetAngle = Math.toDegrees(Math.atan(offsetY / rDistToGoal));

//        if ((isRobotVelocityPos(drive) && isRobotFacingRight(drive) && isRobotRightOfGoal()) ||
//                (isRobotVelocityNeg(drive) && isRobotFacingLeft(drive)) && isRobotRightOfGoal()) {
//            return turretGoalAngleLag = -rTurretGoalAngle - movingTurrOffsetAngle;
//        } else if ((isRobotVelocityPos(drive) && isRobotFacingRight(drive) && isRobotLeftOfGoal()) ||
//                (isRobotVelocityNeg(drive) && isRobotFacingLeft(drive)) && isRobotLeftOfGoal())
//            return turretGoalAngleLag = rTurretGoalAngle - movingTurrOffsetAngle;
//        else if ((isRobotVelocityNeg(drive) && isRobotFacingRight(drive) && isRobotRightOfGoal()) ||
//                (isRobotVelocityPos(drive) && isRobotFacingLeft(drive)) && isRobotRightOfGoal())
//            return turretGoalAngleLag = -rTurretGoalAngle + movingTurrOffsetAngle;
//        else if ((isRobotVelocityNeg(drive) && isRobotFacingRight(drive) && isRobotLeftOfGoal()) ||
//                (isRobotVelocityPos(drive) && isRobotFacingLeft(drive)) && isRobotLeftOfGoal())
//            return turretGoalAngleLag = rTurretGoalAngle + movingTurrOffsetAngle;
//        else
            return rTurretGoalAngle;
    }

    public double getTurretToGoalOffsetAngle() {
        if (Math.abs(rTurretGoalOffsetAngle) > Constants.LIMELIGHT_OFFSET_MAX_DEGREES) {
            rTurretGoalOffsetAngle = Math.signum(rTurretGoalOffsetAngle) * Constants.LIMELIGHT_OFFSET_MAX_DEGREES;
        }
        return rTurretGoalOffsetAngle;
    }

    public void periodic() {
        if (getTurretControlMode() == TurretControlMode.MOTION_MAGIC_TRACK_GYRO) {
            updateGyroTrack();
        }
        else if (getTurretControlMode() == TurretControlMode.MOTION_MAGIC_TRACK_LIMELIGHT) {
            updateLimelightTrack();
        }

        SmartDashboard.putNumber("Turret Angle", this.getTurretAngleAbsoluteDegrees());
        SmartDashboard.putNumber("Lag Angle: ", getTurretToGoalAngle(Drive.getInstance()));
        SmartDashboard.putNumber("Lag Offset Angle: ", getTurretToGoalOffsetAngle());
        SmartDashboard.putNumber("Turret Track Angle: ", Util.normalizeAngle90ToMinus270(getTurretToGoalAngle(Drive.getInstance()) - Drive.getInstance().getGyroFusedHeadingAngleDeg()));
//        SmartDashboard.putNumber("Offset Angle", this.getDriveShootOffSetAngle());
 //       SmartDashboard.putNumber("Turret Angle Ticks", turretMotor.getSelectedSensorPosition());
 //       SmartDashboard.putNumber("Turret Output Percent", turretMotor.getMotorOutputPercent());
 //       SmartDashboard.putNumber("Turret Velocity", turretMotor.getSelectedSensorVelocity());
 //       SmartDashboard.putNumber("Turret Stator Current", turretMotor.getStatorCurrent());
 //       SmartDashboard.putBoolean("Turret Min Sensor", this.getMinTurretSensor());
 //       SmartDashboard.putBoolean("Turret Max Sensor", this.getMaxTurretSensor());
    }

}

