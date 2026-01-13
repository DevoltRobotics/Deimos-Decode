package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

public class PedroSubsystem extends SubsystemBase {

    public final Follower follower;

    public static Pose robotPose = new Pose(0,0,0);

    public PedroSubsystem(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void periodic() {
        follower.update();
        Tuning.follower = follower;
        Tuning.draw();
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }

    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }


    public Command fieldCentricCmd(Gamepad gamepad,Alliance alliance) {
        return new FieldCentricCmd(gamepad, alliance);
    }


    class FieldCentricCmd extends CommandBase {
        Gamepad gamepad;
        Alliance alliance;

        FieldCentricCmd(Gamepad gamepad, Alliance alliance) {
            this.gamepad = gamepad;
            addRequirements(PedroSubsystem.this);
            this.alliance = alliance;
        }

        @Override
        public void initialize() {
            follower.startTeleopDrive();
        }

        @Override
        public void execute() {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x,-gamepad.right_stick_x, false);

            if (gamepad.dpad_up){
                if (alliance == Alliance.RED){
                follower.setPose(new Pose(8.25,7.65,Math.toRadians(90)));} else if (alliance == Alliance.BLUE) {
                    follower.setPose(new Pose(136,7.65,Math.toRadians(90)));
                }else{
                    follower.setPose(new Pose(8.25,7.65,Math.toRadians(90)));
                }
            }
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    class FollowPathCmd extends CommandBase {
        Path path;

        FollowPathCmd(Path path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class FollowPathChainCmd extends CommandBase {
        PathChain path;

        FollowPathChainCmd(PathChain path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path, true);
        }

        @Override
        public void execute() {
            FtcDashboard.getInstance().getTelemetry().addData("FollowPathChainCmd", "Following to " + path.endPoint());
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class HoldPoint extends CommandBase {
        BezierPoint holdPoint;
        double heading;


        public HoldPoint(BezierPoint holdPoint, double heading) {
            this.holdPoint = holdPoint;
            this.heading = heading;
        }

        @Override
        public void initialize() {
            follower.holdPoint(holdPoint, heading);
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }
    }

    class BreakPath extends CommandBase {

        public BreakPath() {
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.breakFollowing();
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

}