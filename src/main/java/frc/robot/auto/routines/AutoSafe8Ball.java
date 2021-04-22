/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoSafe8Ball extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();
    Climb mClimb = Climb.getInstance();

    /**
     * Add your docs here.
     */
    public AutoSafe8Ball() {
        addCommands(
                new ResetOdometryAuto(new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(0))),
                new ExtendIntake(mIntake),
                new ParallelCommandGroup(
                        new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                        new ClimbSetInches(mClimb, -3.0)
                ),
                new ShooterAutoShortShotTrack(mShooter, mMagazine, mTurret, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_6_ROTATIONS),
                new HoodSetAngle(mShooter, Constants. HOOD_RETRACT_HOME_POSITION_DEGREES),
                new RamseteCommand(
                        mTrajectories.getTrench5Ball(),
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
                                mTrajectories.getToMediumFrom5Trench(),
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
                        new ShooterAutoMediumShotTrack(mShooter, mMagazine, mTurret, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL)
                ),
                new StopTrajectory(),
                new ShooterAutoMediumShoot(mShooter,mMagazine,mTurret, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new ShooterReset(mShooter, mMagazine,  Limelight.getInstance(), mTurret)
        );
    }
}
