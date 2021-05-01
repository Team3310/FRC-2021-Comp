package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ResetOdometryAuto extends CommandBase {

    private static final Drive mDrive = Drive.getInstance();
    private Pose2d startPose;

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
