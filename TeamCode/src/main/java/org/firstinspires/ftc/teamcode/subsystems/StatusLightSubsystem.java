package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.light.SetAutoStatus;


public class StatusLightSubsystem extends SubsystemBase {

    Servo light;

    public  StatusLightSubsystem (HardwareMap hardwareMap){
        light = hardwareMap.get(Servo.class,"luz");

    }

    public void SetGreenLight(){
        light.setPosition(0.444);
    }

    public void SetPurpleLight(){
        light.setPosition(0.666);
    }

    public void SetRedLight(){
        light.setPosition(0.279);
    }


}
