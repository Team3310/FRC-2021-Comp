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
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.auto.commands.ShooterAutoLegShoot;
import frc.robot.auto.commands.ShooterAutoLegShotTrack;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.commands.InitializeAuto;
import frc.robot.commands.IntakeExtendAllAuto;
import frc.robot.commands.ShooterReset;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoTrenchSteal8Ball extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();

    public AutoTrenchSteal8Ball() {
        addCommands(
                new ResetOdometryAuto(new Pose2d(Units.inchesToMeters(147), Units.inchesToMeters(-245), new Rotation2d(0))),
                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                new ParallelCommandGroup(
                        new InitializeAuto(mIntake),
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
                       new ShooterAutoLegShotTrack(mShooter, mTurret)
                ),
                new StopTrajectory(),
                new ShooterAutoLegShoot(mShooter,mMagazine,mTurret,
                        Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                new RamseteCommand(
                        mTrajectories.getFirstTwoBallsSteal(),
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
                new StopTrajectory(),
                new ParallelCommandGroup(
                        new RamseteCommand(
                                mTrajectories.getFirstTwoBallsReversedSteal(),
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
                        new ShooterAutoLegShotTrack(mShooter, mTurret)
                ),
                new StopTrajectory(),
                new ShooterAutoLegShoot(mShooter,mMagazine,mTurret,
                        Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new ShooterReset(mShooter, mMagazine,  Limelight.getInstance(), mTurret)
        );
    }
}