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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.auto.commands.ShooterAutoMediumShotTrack;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.commands.IntakeExtendAll;
import frc.robot.commands.IntakeRetractAll;
import frc.robot.commands.ShooterReset;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoTrench6Ball extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();
    Limelight mLimelight = Limelight.getInstance();

    /**
     * Add your docs here.
     */
    public AutoTrench6Ball() {
        addCommands(new SequentialCommandGroup(
                new ResetOdometryAuto(),
                new RamseteCommand(
                                mTrajectories.getToStartTrench(),
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
                new ShooterAutoMediumShotTrack(mShooter,mMagazine,mTurret, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_3_BALL),
                new ParallelDeadlineGroup(
                        new RamseteCommand(
                                mTrajectories.getToTrenchToTrench(),
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
                        new IntakeExtendAll(mIntake, mMagazine)
                ),
                new StopTrajectory(),
                new RamseteCommand(
                mTrajectories.getPanelToTrench(),
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
                mDrive
                ),
                new StopTrajectory(),
                new IntakeRetractAll(mIntake,mMagazine),
                new ShooterAutoMediumShotTrack(mShooter,mMagazine,mTurret, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new ShooterReset(mShooter, mMagazine, mTurret, Limelight.getInstance())
                )
        );
    }
}
