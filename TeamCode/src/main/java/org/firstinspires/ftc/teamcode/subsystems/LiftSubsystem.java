package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

    DcMotor lift;

    public LiftSubsystem(HardwareMap hMap){
        lift = hMap.get(DcMotor.class,"lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up(){
        lift.setPower(1);
    }

    public void down(){
        lift.setPower(-1);
    }

    public void hold(){
        lift.setPower(0);
    }


}
