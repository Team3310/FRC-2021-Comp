/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimbSetHold;
import frc.robot.commands.ClimbSetSpeed;
import frc.robot.commands.IntakeExtendAll;
import frc.robot.commands.IntakeRetractAll;
import frc.robot.commands.IntakeReverseExtendAll;
import frc.robot.commands.IntakeSetSpeed;
import frc.robot.commands.MagazineForward;
import frc.robot.commands.MagazineReverse;
import frc.robot.commands.MagazineSetRPMLimit;
import frc.robot.commands.MagazineSetSpeed;
import frc.robot.commands.ResetAllHomePositions;
import frc.robot.commands.ShooterEject;
import frc.robot.commands.ShooterReset;
import frc.robot.commands.ShooterSetShot;
import frc.robot.commands.ShooterSetTrackShot;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.TurretAutoZero;
import frc.robot.commands.TurretSetToGyroAngle;
import frc.robot.commands.TurretSetToLimelightAngle;
import frc.robot.commands.TurretSetToTrackLimelightAngle;
import frc.robot.controller.GameController;
import frc.robot.controller.Playstation;
import frc.robot.controller.Xbox;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final GameController m_driver = new GameController(Constants.DRIVER_JOYSTICK_1_USB_ID, new Xbox());
    private final GameController m_operator = new GameController(Constants.OPERATOR_JOYSTICK_1_USB_ID, new Playstation());

    private final Intake intake = Intake.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Magazine magazine = Magazine.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Limelight limelight = Limelight.getInstance();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Pass the driver controller to the drive subsystem for teleop control
        drive.setDriverController(m_driver);

        configureButtonBindings();
    }

    public static Trajectory loadPathTrajectory(String JSONPath) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + JSONPath, ex.getStackTrace());
        }
        return trajectory;
    }
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {
        
        // Operator
        Button shooterBumper = m_operator.getRightBumper();
        shooterBumper.whenPressed(new ShooterShoot(shooter, magazine, turret, limelight));
        shooterBumper.whenReleased(new ShooterReset(shooter, magazine, limelight, turret));

        Button shooterEjectButton = m_operator.getLeftBumper();
        shooterEjectButton.whenPressed(new ShooterEject(shooter));
        shooterEjectButton.whenReleased(new ShooterReset(shooter, magazine, limelight, turret));

        Button intakeTrigger = m_operator.getRightTrigger();
        intakeTrigger.whenPressed(new IntakeExtendAll(intake, magazine));
        intakeTrigger.whenReleased(new IntakeRetractAll(intake, magazine));

        Button intakeReverseTrigger = m_operator.getLeftTrigger();
        intakeReverseTrigger.whenPressed(new IntakeReverseExtendAll(intake, magazine));
        intakeReverseTrigger.whenReleased(new IntakeSetSpeed(intake, 0));

        Button longShotButton = m_operator.getButtonY();
        ShooterSetShot.ShooterParams paramsLong = new ShooterSetShot.ShooterParams();
        paramsLong.limelightOffset = Constants.LIMELIGHT_OFFSET_LONG_SHOT_DEGREES;
        paramsLong.hoodAngle = Constants.HOOD_LONG_ANGLE_DEGREES;
        paramsLong.kickerRPM = Constants.SHOOTER_KICKER_LONG_RPM;
        paramsLong.limelightPipeline = Constants.LIMELIGHT_LONG_PIPELINE;
        paramsLong.shooterRPM = Constants.SHOOTER_MAIN_LONG_RPM;
        paramsLong.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_LONG_SHOT_ANGLE_DEGREES;
        longShotButton.whenPressed(new ShooterSetShot(shooter, magazine, turret, paramsLong));

        Button mediumShotButton = m_operator.getButtonB();
        ShooterSetShot.ShooterParams paramsMedium = new ShooterSetShot.ShooterParams();
        paramsMedium.limelightOffset = Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES;
        paramsMedium.hoodAngle = Constants.HOOD_MEDIUM_ANGLE_DEGREES;
        paramsMedium.kickerRPM = Constants.SHOOTER_KICKER_MEDIUM_RPM;
        paramsMedium.limelightPipeline = Constants.LIMELIGHT_MEDIUM_PIPELINE;
        paramsMedium.shooterRPM = Constants.SHOOTER_MAIN_MEDIUM_RPM;
        paramsMedium.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES;
        mediumShotButton.whenPressed(new ShooterSetShot(shooter, magazine, turret, paramsMedium));

        Button autoShotButton = m_operator.getButtonX();
        ShooterSetTrackShot.ShooterParams paramsAll = new ShooterSetTrackShot.ShooterParams();
        paramsAll.limelightOffset = 0;
        paramsAll.limelightPipeline = Constants.LIMELIGHT_ALL_FIELD_PIPELINE;
        autoShotButton.whenPressed(new ShooterSetTrackShot(shooter, magazine, turret, paramsAll, drive));

        Button keyShotButton = m_operator.getButtonA();
        ShooterSetShot.ShooterParams paramsKey = new ShooterSetShot.ShooterParams();
        paramsKey.limelightOffset = Constants.LIMELIGHT_OFFSET_KEY_SHOT_DEGREES;
        paramsKey.hoodAngle = Constants.HOOD_KEY_ANGLE_DEGREES;
        paramsKey.kickerRPM = Constants.SHOOTER_KICKER_KEY_RPM;
        paramsKey.isLimelightActive = false;
        paramsKey.shooterRPM = Constants.SHOOTER_MAIN_KEY_RPM;
        paramsKey.turretGyroOffset = Constants.TURRET_GYRO_OFFSET_KEY_SHOT_ANGLE_DEGREES;
        keyShotButton.whenPressed(new ShooterSetShot(shooter, magazine, turret, paramsKey));

        Button climbUpButton = m_operator.getDPadUp();
        climbUpButton.whenPressed(new ClimbSetSpeed(climb, 1.0));
        climbUpButton.whenReleased(new ClimbSetHold(climb));

        Button climbDownButton = m_operator.getDPadDown();
        climbDownButton.whenPressed(new ClimbSetSpeed(climb, -1.0));
        climbDownButton.whenReleased(new ClimbSetHold(climb));

        Button magazineForwardButton = m_operator.getDPadLeft();
        magazineForwardButton.whenPressed(new MagazineForward(intake, magazine));
        magazineForwardButton.whenReleased(new SequentialCommandGroup(new MagazineSetSpeed(magazine, 0), new IntakeSetSpeed(intake,0)));

        Button magazineReverseButton = m_operator.getDPadRight();
        magazineReverseButton.whenPressed(new MagazineReverse(intake, magazine));
        magazineReverseButton.whenReleased(new SequentialCommandGroup(new MagazineSetSpeed(magazine, 0), new IntakeSetSpeed(intake,0)));

        // Driver
        Button resetHomeButton = m_driver.getStartButton();
        resetHomeButton.whenPressed(new ResetAllHomePositions(drive, turret, magazine, shooter, climb));

        Button turretAimToGoal = m_driver.getButtonA();
        turretAimToGoal.whenPressed(new InstantCommand(() ->turret.setPositionToGoalAngle(drive)));

        Button limelightTrack = m_driver.getButtonX();
        limelightTrack.whenPressed(new TurretSetToTrackLimelightAngle(turret, 0, true));

        // Button driveGyroResetButton = m_driver.getButtonY();
        // driveGyroResetButton.whenPressed(new SequentialCommandGroup(
        //         new InstantCommand(() -> drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES)),
        //         new TurretAutoZero(turret)
        // ));
        Button driveGyroTurnButton = m_driver.getButtonY();
        driveGyroTurnButton.whenPressed(new TurretSetToGyroAngle(turret, 0));

        Button limelightTrackTwo = m_driver.getButtonB();
        limelightTrackTwo.whenPressed( new TurretSetToLimelightAngle(turret, 0));
 
//        SmartDashboard.putData("Reset All Home", new ResetAllHomePositions(drive, turret, magazine, shooter));
//        SmartDashboard.putData("Compressor On", new InstantCommand(() -> compressor.turnCompressorOn()));
//        SmartDashboard.putData("Compressor Off", new InstantCommand(() -> compressor.turnCompressorOff()));
//        SmartDashboard.putData("Climb Arm Lock", new InstantCommand(() -> intake.climbLock()));
//        SmartDashboard.putData("Climb Arm Release", new InstantCommand(() -> intake.climbRelease()));
//        SmartDashboard.putData("Intake Inner Extend", new InstantCommand(() -> intake.extendIntakeInnerArms()));
//        SmartDashboard.putData("Intake OuterExtend", new InstantCommand(() -> intake.extendIntakeOuterArms()));
//
//        SmartDashboard.putData("Climb PTO Lock", new InstantCommand(() -> intake.climbPTOLock()));
//        SmartDashboard.putData("Climb PTO Engage", new InstantCommand(() -> intake.climbPTOEngage()));
//
        SmartDashboard.putData("Limelight LED off", new InstantCommand(() -> limelight.setLedMode(Limelight.LightMode.OFF)));
        SmartDashboard.putData("Limelight LED on", new InstantCommand(() -> limelight.setLedMode(Limelight.LightMode.ON)));

//        SmartDashboard.putData("Intake Set Speed", new InstantCommand(()-> intake.setRollerSpeed(0.2)));
//        SmartDashboard.putData("Intake Set Speed OFF", new InstantCommand(() -> intake.setRollerSpeed(0.0)));
//        SmartDashboard.putData("Intake Set RPM", new InstantCommand(() -> intake.setRollerRPM(2000.0)));

        SmartDashboard.putData("Mag Set Speed", new InstantCommand(()-> magazine.setMagazineSpeed(0.2)));
        SmartDashboard.putData("Mag Set Speed OFF", new InstantCommand(() -> magazine.setMagazineSpeed(0.0)));
//        SmartDashboard.putData("Mag Set RPM", new InstantCommand(()-> magazine.setMagazineRPM(60.0)));
        SmartDashboard.putData("Mag Set RPM Limit", new MagazineSetRPMLimit(magazine, 40, 20));
        SmartDashboard.putData("Mag Set MM", new InstantCommand(()-> magazine.setMagazineMotionMagicPositionAbsolute(-180.0 + 72.0)));
//        SmartDashboard.putData("Mag Reset", new InstantCommand(() -> magazine.resetHomePosition()));
//        SmartDashboard.putData("Mag Index Divider", new MagazineIndexDividerToTurret(magazine, turret));
        SmartDashboard.putData("Shooter Reset", new ShooterReset(shooter, magazine, limelight, turret));

        SmartDashboard.putData("Shooter Main Set Speed", new InstantCommand(()-> shooter.setMainSpeed(0.2)));
//        SmartDashboard.putData("Shooter Main Set OFF", new InstantCommand(()-> shooter.setMainSpeed(0.0)));
//        SmartDashboard.putData("Shooter Main Set RPM Fender", new InstantCommand(()-> shooter.setMainRPM(2100)));
//        SmartDashboard.putData("Shooter Main Set RPM Auton", new InstantCommand(()-> shooter.setMainRPM(3500)));
//        SmartDashboard.putData("Shooter Main Set RPM Long", new InstantCommand(()-> shooter.setMainRPM(4700)));
//
//        SmartDashboard.putData("Shooter Kicker Set Speed", new InstantCommand(()-> shooter.setKickerSpeed(0.2)));
//        SmartDashboard.putData("Shooter Kicker Set OFF", new InstantCommand(()-> shooter.setKickerSpeed(0.0)));
//        SmartDashboard.putData("Shooter Kicker Set RPM Fender", new InstantCommand(()-> shooter.setKickerRPM(2100)));
//        SmartDashboard.putData("Shooter Kicker Set RPM Auton", new InstantCommand(()-> shooter.setKickerRPM(3500)));
//        SmartDashboard.putData("Shooter Kicker Set RPM Long", new InstantCommand(()-> shooter.setKickerRPM(4700)));
//
//        SmartDashboard.putData("Shooter Intake Set Speed", new InstantCommand(()-> shooter.setIntakeSpeed(0.2)));
//        SmartDashboard.putData("Shooter Intake Set OFF", new InstantCommand(()-> shooter.setIntakeSpeed(0.0)));
//        SmartDashboard.putData("Shooter Intake Set RPM Reverse", new InstantCommand(()-> shooter.seIntakeRPM(-300)));
//        SmartDashboard.putData("Shooter Intake Set RPM Auto", new InstantCommand(() -> shooter.seIntakeRPM(3000)));
//        SmartDashboard.putData("Shooter Intake Set RPM Long", new InstantCommand(()-> shooter.seIntakeRPM(4000)));

//        SmartDashboard.putData("Turret Set Speed", new InstantCommand(()-> turret.setTurretSpeed(0.2)));
//        SmartDashboard.putData("Turret Set OFF", new InstantCommand(() -> turret.setTurretSpeed(0.0)));
//        SmartDashboard.putData("Turret Reset", new InstantCommand(() -> turret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES)));
//        SmartDashboard.putData("Turret MM", new InstantCommand(() -> turret.setTurretMotionMagicPositionAbsolute(-135)));
//        SmartDashboard.putData("Turret MM 180", new InstantCommand(() -> turret.setTurretMotionMagicPositionAbsolute(-180)));
//        SmartDashboard.putData("Turret Position", new InstantCommand(()-> turret.setTurretPositionRelative(5)));
//        SmartDashboard.putData("Turret Position Neg", new InstantCommand(()-> turret.setTurretPositionRelative(-5)));
//          SmartDashboard.putData("Turret Auto Zero", new TurretAutoZero(turret));
//        SmartDashboard.putData("Turret Turn Gyro", new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES));
//        SmartDashboard.putData("Turret Turn Limelight", new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES));

//        SmartDashboard.putData("Shoot Medium Shot", new ShooterMediumShot(shooter, magazine, turret));
//        SmartDashboard.putData("Shoot Auto Shot", new ShooterAutoShot(shooter, magazine, turret));
//        SmartDashboard.putData("Shoot Long Shot", new ShooterLongShot(shooter, magazine, turret));
//        SmartDashboard.putData("Shoot Fender Shot", new ShooterFenderShot(shooter, magazine, turret));

//        SmartDashboard.putData("Intake RPM reverse", new IntakeSetRPM(intake, -300));
//        SmartDashboard.putData("Intake Extend All", new IntakeExtendAll(intake, magazine));
//        SmartDashboard.putData("Intake Retract All", new IntakeRetractAll(intake, magazine));
//        SmartDashboard.putData("Intake Extend Inner", new InstantCommand(() -> intake.extendIntakeInnerArms()));
//        SmartDashboard.putData("Intake Extend Outer", new InstantCommand(() -> intake.extendIntakeOuterArms()));
//        SmartDashboard.putData("Intake Retract All", new IntakeRetractAll(intake, magazine));
//        SmartDashboard.putData("Climb Reset Encoder", new InstantCommand(() -> intake.resetIntakeEncoder()));
//        SmartDashboard.putData("Climb MM", new InstantCommand(() -> intake.setClimbMotionMagicPositionAbsolute(10)));

//        SmartDashboard.putData("Hood Set Forward", new InstantCommand(()-> shooter.setHoodSpeed(0.3)));
//        SmartDashboard.putData("Hood Set OFF", new InstantCommand(() -> shooter.setHoodSpeed(0.0)));
//        SmartDashboard.putData("Hood Reset", new InstantCommand(() -> shooter.resetHoodHomePosition()));
//        SmartDashboard.putData("Hood MM", new HoodSetAngle(shooter, Constants.HOOD_TEST_ANGLE_DEGREES));
//        SmartDashboard.putData("Hood MM minus 5", new HoodSetAngle(shooter, Constants.HOOD_TEST_ANGLE_DEGREES-5));
//        SmartDashboard.putData("Hood MM V2", new HoodSetAngle(shooter, 2));

//        SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES)));
//        SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> drive.resetEncoders()));

        SmartDashboard.putData("Reset All Home", new ResetAllHomePositions(drive, turret, magazine, shooter, climb));
        SmartDashboard.putData("Reset Gyro", new InstantCommand(() ->
                drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES)));
        SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> drive.resetEncoders()));
        SmartDashboard.putData("Reset Climb Encoders", new InstantCommand(() -> climb.resetClimbEncoder()));
        SmartDashboard.putData("Climb MM Down", new InstantCommand(() ->climb.setClimbMotionMagicPositionAbsolute(-3)));
        SmartDashboard.putData("Climb MM Zero", new InstantCommand(() ->climb.setClimbMotionMagicPositionAbsolute(0)));
        SmartDashboard.putData("Climb MM Up", new InstantCommand(() ->climb.setClimbMotionMagicPositionAbsolute(20)));
    }
}
