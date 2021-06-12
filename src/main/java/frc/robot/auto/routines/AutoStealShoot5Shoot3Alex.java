package frc.robot.auto.routines;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.*;
import frc.robot.commands.InitializeAuto;
import frc.robot.commands.IntakeExtendAllAuto;
import frc.robot.commands.ShooterReset;
import frc.robot.commands.ShooterSetShot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoStealShoot5Shoot3Alex extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();
    Climb mClimb = Climb.getInstance();

    public AutoStealShoot5Shoot3Alex() {

        ShooterSetShot.ShooterParams paramsLeg = new ShooterSetShot.ShooterParams();
        paramsLeg.limelightOffset = Constants.LIMELIGHT_OFFSET_LEG_SHOT_DEGREES;
        paramsLeg.hoodAngle = Constants.HOOD_LEG_ANGLE_DEGREES;
        paramsLeg.kickerRPM = Constants.SHOOTER_KICKER_LEG_RPM;
        paramsLeg.limelightPipeline = Constants.LIMELIGHT_LEG_PIPELINE;
        paramsLeg.shooterRPM = Constants.SHOOTER_MAIN_LEG_RPM;
        paramsLeg.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_LEG_SHOT_ANGLE_DEGREES;

        ShooterSetShot.ShooterParams paramsBar = new ShooterSetShot.ShooterParams();
        paramsBar.limelightOffset = Constants.LIMELIGHT_OFFSET_BAR_DEGREES;
        paramsBar.hoodAngle = Constants.HOOD_BAR_ANGLE_DEGREES;
        paramsBar.kickerRPM = Constants.SHOOTER_KICKER_BAR_RPM;
        paramsBar.limelightPipeline = Constants.LIMELIGHT_BAR_PIPELINE;
        paramsBar.shooterRPM = Constants.SHOOTER_BAR_RPM;
        paramsBar.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_BAR_SHOT_ANGLE_DEGREES;


        addCommands(
                new ResetOdometryAuto(new Pose2d(Units.inchesToMeters(147), Units.inchesToMeters(-245), new Rotation2d(0))),
                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                new ParallelCommandGroup(
                        new InitializeAuto(mIntake, mClimb),
                        new RamseteCommand(
                        mTrajectories.getStealStartToStealBallV2(),
                        mDrive::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        mDrive::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        mDrive::tankDriveVolts,
                        mDrive)
                        ),
                new StopTrajectory(),
                new ParallelCommandGroup(
                        new RamseteCommand(
                                mTrajectories.getStealBallToCenterShotV2(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),
                        new ShooterSetShot(mShooter, mMagazine, mTurret, paramsLeg)
                       //new ShooterAutoLegShotTrack(mShooter, mTurret)
                ),
                new StopTrajectory(),
                new ShooterShootAuto(mShooter, mMagazine, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL)
//                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
//                new RamseteCommand(
//                        mTrajectories.getFirstTwoBallsSteal(),
//                        mDrive::getPose,
//                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
//                        new SimpleMotorFeedforward(Constants.ksVolts,
//                                Constants.kvVoltSecondsPerMeter,
//                                Constants.kaVoltSecondsSquaredPerMeter),
//                        Constants.kDriveKinematics,
//                        mDrive::getWheelSpeeds,
//                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
//                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
//                        // RamseteCommand passes volts to the callback
//                        mDrive::tankDriveVolts,
//                        mDrive),
//                new StopTrajectory(),
//                new ParallelCommandGroup(
//                        new RamseteCommand(
//                                mTrajectories.getFirstTwoBallsReversedSteal(),
//                                mDrive::getPose,
//                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
//                                new SimpleMotorFeedforward(Constants.ksVolts,
//                                        Constants.kvVoltSecondsPerMeter,
//                                        Constants.kaVoltSecondsSquaredPerMeter),
//                                Constants.kDriveKinematics,
//                                mDrive::getWheelSpeeds,
//                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
//                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
//                                // RamseteCommand passes volts to the callback
//                                mDrive::tankDriveVolts,
//                                mDrive)
//                ),
//                new StopTrajectory(),
//                new ShooterSetShot(mShooter, mMagazine,mTurret,paramsBar),
//                new ShooterShootAuto(mShooter, mMagazine, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
//                new ShooterReset(mShooter, mMagazine,  Limelight.getInstance(), mTurret)
        );
    }
}