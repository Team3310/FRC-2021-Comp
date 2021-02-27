package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;

public class ResetOdometryAuto extends CommandBase {

    private static final Drive mDrive = Drive.getInstance();
    private static final Turret mTurret = Turret.getInstance();
    private Pose2d startPose;
    private static final Climb mClimb = Climb.getInstance();

    public ResetOdometryAuto(Pose2d startPose) {
        this.startPose = startPose;
    }

    public ResetOdometryAuto() {
        this.startPose = Constants.ROBOT_START_POSITION;;
    }

    @Override
    public void initialize() {
        System.out.println("Reset Odometry Started");
        mDrive.resetOdometry(startPose);
        mDrive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES);
        mTurret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES);
        mClimb.resetClimbEncoder();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Reset Odometry Finished");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
