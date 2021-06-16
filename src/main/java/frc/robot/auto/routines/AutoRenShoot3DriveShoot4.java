package frc.robot.auto.routines;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoRenShoot3DriveShoot4 extends SequentialCommandGroup {

    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();
    Climb mClimb = Climb.getInstance();
    Limelight mLimelight = Limelight.getInstance();

    public AutoRenShoot3DriveShoot4() {

        ShooterSetShot.ShooterParams paramsLeg = new ShooterSetShot.ShooterParams();
        paramsLeg.limelightOffset = Constants.LIMELIGHT_OFFSET_LEG_SHOT_DEGREES;
        paramsLeg.hoodAngle = Constants.HOOD_LEG_ANGLE_DEGREES;
        paramsLeg.kickerRPM = Constants.SHOOTER_KICKER_LEG_RPM;
        paramsLeg.limelightPipeline = Constants.LIMELIGHT_LEG_PIPELINE;
        paramsLeg.shooterRPM = Constants.SHOOTER_MAIN_LEG_RPM;
        paramsLeg.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_LEG_SHOT_ANGLE_DEGREES;

        ShooterSetShot.ShooterParams paramsMedium = new ShooterSetShot.ShooterParams();
        paramsMedium.limelightOffset = 0;
        paramsMedium.hoodAngle = Constants.HOOD_MEDIUM_ANGLE_DEGREES;
        paramsMedium.kickerRPM = Constants.SHOOTER_KICKER_MEDIUM_RPM;
        paramsMedium.limelightPipeline = Constants.LIMELIGHT_MEDIUM_PIPELINE;
        paramsMedium.shooterRPM = Constants.SHOOTER_MAIN_MEDIUM_RPM;
        paramsMedium.turretGyroOffset = 0;

        ShooterSetShot.ShooterParams paramsShort = new ShooterSetShot.ShooterParams();
        paramsShort.limelightOffset = Constants.LIMELIGHT_OFFSET_AUTO_SHOT_DEGREES;
        paramsShort.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_AUTON_SHORT_SHOT_ANGLE_DEGREES;
        paramsShort.magazineRPM = Constants.MAGAZINE_SHOOT_RPM;
        paramsShort.kickerRPM = Constants.SHOOTER_KICKER_AUTON_SHORT_RPM;
        paramsShort.shooterRPM = Constants.SHOOTER_MAIN_AUTON_SHORT_RPM;
        paramsShort.hoodAngle = Constants.HOOD_AUTON_SHORT_ANGLE_DEGREES;

        addCommands(
                new ResetAllHomeAuto(mDrive, mTurret, mMagazine, mShooter, mClimb, new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(0))),
                new ExtendIntake(mIntake),
                new ShooterSetShot(mShooter, mMagazine, mTurret, paramsShort),
                new ShooterShootAuto(mShooter, mMagazine, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),

                new ParallelCommandGroup(
                        new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                        new ClimbSetInches(mClimb, -3.0)
                ),
                new RamseteCommand(
                        mTrajectories.getFirstFourBalls(),
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
                new WaitCommand(1.0),
                new RamseteCommand(
                        mTrajectories.reverseToShootFromFour(),
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

                new ShooterSetShot(mShooter, mMagazine, mTurret, paramsMedium),
                new ShooterShootAuto(mShooter, mMagazine, Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL)
        );

    }
}


