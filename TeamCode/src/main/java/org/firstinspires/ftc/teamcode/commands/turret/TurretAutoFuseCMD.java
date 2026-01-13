package org.firstinspires.ftc.teamcode.commands.turret;

import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;


public class TurretAutoFuseCMD extends TurretToPosCMD {

    public static double tX;

    private final TurretSubsystem turret;
    private final LLSubsystem ll;

    private final MovingStatistics llAverage = new MovingStatistics(5);

    public TurretAutoFuseCMD(TurretSubsystem subsystem, LLSubsystem llSubsystem) {
        super(subsystem, 0d, false);
        this.turret = subsystem;
        this.ll = llSubsystem;

        // require both so nothing else fights LL pipeline / turret
        addRequirements(subsystem, llSubsystem);
    }

    @Override
    public void initialize() {
        ll.setAimingPipeline();
        llAverage.clear();
    }

    @Override
    public void execute() {
        ll.setAimingPipeline();

        // If LL valid, feed measurement to subsystem KF
        if (ll.result != null && ll.result.isValid()) {
            Double tx = ll.getAllianceTX();

            if (tx != null) {
                // keep your alliance adjustment
                turret.pushLimelightMeasurement(tx);
            }
        }

        // Always aim using fused setpoint (KF already predicted from odo inside periodic)
        targetPos = turret.getFusedTurretGoalAngle();
        super.execute();
    }
}
