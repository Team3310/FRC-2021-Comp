package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Util;

public class Climb extends SubsystemBase {

    // Conversions
    private static final double CLIMB_OUTPUT_TO_ENCODER_RATIO = (15.0 / 4.0) * (15.0 / 7.0) * (15.0 / 8.0);
    public static final double CLIMB_DRUM_DIAMETER_INCHES = 1.25;
    public static final double CLIMB_REVOLUTIONS_TO_INCHES = Math.PI * CLIMB_DRUM_DIAMETER_INCHES;
    public static final double CLIMB_INCHES_TO_ENCODER_TICKS = CLIMB_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / CLIMB_REVOLUTIONS_TO_INCHES;


    // Motor Controllers
    private TalonFX climbMotor;

    // Misc
    private static final int kClimbMotionMagicSlot = 1;
    private double targetPositionTicks = 0;

    private final static Climb INSTANCE = new Climb();

    private Climb() {
        climbMotor = new TalonFX(Constants.CLIMBER_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        climbMotor.configAllSettings(configs);

        climbMotor.setInverted(TalonFXInvertType.CounterClockwise);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configMotionCruiseVelocity(6000);
        climbMotor.configMotionAcceleration(14000);
        climbMotor.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 40;
        statorCurrentConfigs.enable = true;
        climbMotor.configStatorCurrentLimit(statorCurrentConfigs);

        climbMotor.config_kF(kClimbMotionMagicSlot, 0.0);
        climbMotor.config_kP(kClimbMotionMagicSlot, 0.2);
        climbMotor.config_kI(kClimbMotionMagicSlot, 0.0);
        climbMotor.config_kD(kClimbMotionMagicSlot, 0.0);
        climbMotor.config_IntegralZone(kClimbMotionMagicSlot, CLIMB_INCHES_TO_ENCODER_TICKS * 5);
    }

    public static Climb getInstance() {
        return INSTANCE;
    }


    // Motion Magic
    public synchronized void setClimbMotionMagicPositionAbsolute(double inches) {
        climbMotor.selectProfileSlot(kClimbMotionMagicSlot, 0);
        targetPositionTicks = getClimbEncoderTicksAbsolute(limitClimbInches(inches));
        climbMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return Util.epsilonEquals(climbMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    private int getClimbEncoderTicksAbsolute(double inches) {
        return (int) (inches * CLIMB_INCHES_TO_ENCODER_TICKS);
    }

    public double getClimbAbsoluteInches() {
        return (double) climbMotor.getSelectedSensorPosition() / CLIMB_INCHES_TO_ENCODER_TICKS;
    }

    private double limitClimbInches(double targetInches) {
        if (targetInches < Constants.CLIMB_MIN_INCHES) {
            return Constants.CLIMB_MIN_INCHES;
        } else if (targetInches > Constants.CLIMB_MAX_INCHES) {
            return Constants.CLIMB_MAX_INCHES;
        }

        return targetInches;
    }

    public void resetClimbEncoder() {
        this.climbMotor.setSelectedSensorPosition(0);
    }

    public double getClimbRotations() {
        return climbMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / CLIMB_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getClimbInches() {
        return getClimbRotations() * CLIMB_REVOLUTIONS_TO_INCHES;
    }

    public void periodic() {
        SmartDashboard.putNumber("Climb Inches", this.getClimbInches());
    }
}
