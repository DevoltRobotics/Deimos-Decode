package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.LoggedSubsystem;
import org.firstinspires.ftc.teamcode.config.PosUtils;


@Config
public class TurretSubsystem extends LoggedSubsystem {

    Servo Torreta;

    Translation2d RotatedTurretOffset;
    Translation2d TurretFieldPos;

    Translation2d RobotPos;
    public static double lowerLimit = -110;
    public static double upperLimit = 207.0642;

    public double TurrettargetDeg = 0;



    Translation2d TurretRobotPos = new Translation2d(-1.22,0);



    double goalX, goalY;

    Pose TurretPos;

    Pose FTCPos;
    public static double currentRelativePos;

    PedroSubsystem pedroSubsystem;

    double robotToGoalAngle;
    double turretToGoalAngle;

     public double distanceToGoal = 0;

     public double heading;

     public double offsetY;
     public double offsetX;

     public static double kF = 0.02;

     public static double LeftkF = 0.016;

    Alliance alliance;

    Translation2d GoalPos;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public TurretSubsystem(HardwareMap hMap, PedroSubsystem pedroSubsystem, Alliance alliance) {

        Torreta = hMap.get(Servo.class, "torreta");
        Torreta.setDirection(Servo.Direction.REVERSE);

        this.pedroSubsystem = pedroSubsystem;
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setGoalPos(144, 144);
            GoalPos = new Translation2d(goalX,goalY);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(0, 144);
            GoalPos = new Translation2d(goalX,goalY);
        } else {
            setGoalPos(130, 130);
            GoalPos = new Translation2d(goalX,goalY);

        }

    }



    @Override
    public void periodic() {
        Pose robotPos = pedroSubsystem.follower.getPose();
        RobotPos = new Translation2d(robotPos.getX(),robotPos.getY());

        heading = robotPos.getHeading();
        Rotation2d rotation2d = new Rotation2d(heading);

        RotatedTurretOffset = TurretRobotPos.rotateBy(rotation2d);

        TurretFieldPos = RobotPos.plus(RotatedTurretOffset);

        double dx = goalX - TurretFieldPos.getX();
        double dy = goalY - TurretFieldPos.getY();

        Translation2d TurrettoGoal = GoalPos.minus(TurretFieldPos);

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy,dx));//direction
        distanceToGoal = Math.hypot(dx,dy);//magnitude

        turretToGoalAngle = AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle);

        TurretPos = new Pose(TurretFieldPos.getX(), TurretFieldPos.getY(), Math.toRadians(robotToGoalAngle));

        FTCPos = PosUtils.PedrotoFTC(TurretPos);



      /*  FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("Servo pos", Torreta.getPosition() );*/

    }

    public void TurretSetPos(double PosDeg){
        double servoPos = (PosDeg - lowerLimit) / (upperLimit - lowerLimit);
        if (PosDeg > 0) {
            servoPos += kF;
        }else if (PosDeg < 0) {
            servoPos -= LeftkF;
        }
        Torreta.setPosition(Range.clip(servoPos,0,1));

    }

    public double convertDegreestoServoPos(double deg){
        return deg*0.0031539354+0.35;
    }

    public void SetServopos (double pos){
        Torreta.setPosition(pos);
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }


    @Override
    public void log(TelemetryPacket packet) {
        packet.put("Turret/Pose x", FTCPos.getX());
        packet.put("Turret/Pose y", FTCPos.getY());
        packet.put("Turret/Pose heading", FTCPos.getHeading());
        packet.put("Turret/Targetpos", TurrettargetDeg);
    }
}
