package org.firstinspires.ftc.teamcode.TeleOp.drive;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.BASE_POSE_TOLERANCE;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.CurrentAlliance;
import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.util.BooleanTrigger;
import org.firstinspires.ftc.teamcode.util.CommandUtils.Command;
import org.firstinspires.ftc.teamcode.util.CommandUtils.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;

public class PedroDrive implements SubsystemInternal {

    public Follower follower;
    private BetterGamepad controller1;
    private BetterGamepad controller2;

    private CurrentAlliance alliance;

    public void provideInitComponents(Follower follower, BetterGamepad controller1, BetterGamepad controller2, CurrentAlliance alliance) {

        this.follower = follower;

        this.controller1 = controller1;
        this.controller2 = controller2;

        this.alliance = alliance;
    }

    private Intake.Stage stage;

    public void provideLoopComponents(Intake.Stage stage) {
        this.stage = stage;
    }

    private boolean driveToBaseAllowed = true;

    private boolean teleOpDrive = false;

    public void update() {

        if (stage == Intake.Stage.DRIVE_TO_BASE && driveToBaseAllowed) {

            if (controller1.right_stick_buttonHasJustBeenPressed || controller2.left_stick_buttonHasJustBeenPressed) {

                follower.breakFollowing();
                driveToBaseAllowed = false;
                teleOpDrive = false;
            }
            else {

                if (follower.atPose(Constants.DriveConstants.getBasePose(alliance), BASE_POSE_TOLERANCE[0], BASE_POSE_TOLERANCE[1], BASE_POSE_TOLERANCE[2])) {
                    if (!follower.isBusy()) {
                        driveToBaseAllowed = false;
                        teleOpDrive = false;
                    }
                }
                else {
                    follower.followPath(Constants.DriveConstants.getMoveToBasePathChain(alliance, follower), false);
                }
            }
        }
        else {

            if (stage != Intake.Stage.DRIVE_TO_BASE) {
                driveToBaseAllowed = true;
            }

            if (!teleOpDrive) {
                follower.breakFollowing();
                follower.startTeleOpDrive(true);
                teleOpDrive = true;
            }
            follower.setTeleOpDrive(-controller1.left_stick_y(), -controller1.left_stick_x(), -controller1.right_stick_x());
        }
    }

}