package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public double AngleError;

    HookSubsystem hookSubsystem;

    DigitalChannel laser;
    public int lastFlickSeen = Integer.MIN_VALUE;
    CRServo spindex;

    public boolean BPresence = false;

    public NormalizedColorSensor colorSensor;
    AnalogInput spindexEncoder;
    public double SpindexPos;

    public PIDFController spindexPID;
    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(
            0.5,   // P
            0.00,   // I
            0.015,  // D
            0.00    // F
    );

    public static double DEADBAND = Math.toRadians(1);

    public static double Min_Output = 0.043;
    private final ElapsedTime timer = new ElapsedTime();

    public static double TRIGGER_COOLDOWN_MS = 250;             // anti rebote tiempo
    public static double MIN_ADVANCE_RAD = Math.toRadians(110); // avance mínimo para indexar

    private double targetPosRad = 0.0;

    private boolean Shootmode = false;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    public static int nBalls = 0; // CONTAODR INTERNO
    private double deltaT;

    public enum DetectedColor {PURPLE, GREEN, UNKNOWN}

    double NextPosRad = Math.toRadians(120);
    public double IntakePos = 0.6549744683847812;
    public double ShootPos = 0.4798068780028048;

    public boolean FirstInitIn = true;

    public boolean FirstInitSho = true;

    //funciones para CMDS

    public boolean getFirstInitIn(){
        return FirstInitIn;
    }

    public void setFirstInitIn(boolean FirstInitIn){
        this.FirstInitIn = FirstInitIn;
    }

    public boolean getFirstInitSho(){
        return FirstInitSho;
    }

    public void setFirstInitsho(boolean FirstInitSho){
        this.FirstInitSho = FirstInitSho;
    }

    public void setShootmode(boolean Shootmode){
        this.Shootmode = Shootmode;
    }

    public void holdShootmode(){
        Shootmode = true;
    }

    public boolean getShootmode(){
        return Shootmode;
    }

    public void advanceOneIndex() {
        targetPosRad = normalizeRadians(targetPosRad - NextPosRad);
    }

    public void returnOneIndex(){
        targetPosRad = normalizeRadians(targetPosRad+NextPosRad);
    }

    public int getnBalls(){
        return nBalls;
    }

    public void addnBalls(){
        nBalls ++;
    }

    public void lessBalls(){
        nBalls --;
    }

    public void resetnBalls(){
        nBalls = 0;
    }

    public double getAngleError(){
        return AngleError;
    }

    public void setTargetPosRad(double targetPosRad){
        this.targetPosRad = targetPosRad;
    }

    public double getSpindexPos(){
        return SpindexPos;
    }

    public boolean getBPresence(){
        return BPresence;
    }

    public double getAngleDiff(double a, double b){
        return Math.abs(MathFunctions.getSmallestAngleDifference(a, b));
    }

    public SpindexSubsystem(HardwareMap hMap, HookSubsystem hookSubsystem) {
        this.hookSubsystem = hookSubsystem;

        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        spindex = hMap.get(CRServo.class, "spindex");


        spindexEncoder = hMap.get(AnalogInput.class, "Spencoder");

        colorSensor = hMap.get(NormalizedColorSensor.class, "colorL");
        colorSensor.setGain(8);

        spindexPID = new PIDFController(SPcoeffs);


        timer.reset();

        lastFlickSeen = hookSubsystem.nFlick;

        spindexPID.setTolerance(DEADBAND);
    }

    @Override
    public void periodic(){
        spindexPID.setCoefficients(SPcoeffs);

        if (nBalls == 3){
            Shootmode = true;
        }

        if (nBalls == 0){
            Shootmode = false;
        }

        if (nBalls < 0){
            nBalls = 0;
        } else if (nBalls > 3) {
            nBalls = 3;
        }


        BPresence = laser.getState();

        SpindexPos = (spindexEncoder.getVoltage() / 3.3) * 2 * Math.PI;
        SpindexPos = normalizeRadians(SpindexPos);
        targetPosRad = normalizeRadians(targetPosRad);

        AngleError =
                MathFunctions.getTurnDirection(targetPosRad, SpindexPos) *
                        MathFunctions.getSmallestAngleDifference(targetPosRad, SpindexPos);

        spindexPID.setSetPoint(0);
        double power = spindexPID.calculate(AngleError);

        spindex.setPower(power);


        FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex crudo", SpindexPos);
        FtcDashboard.getInstance().getTelemetry().addData("target pos sp", targetPosRad);
        FtcDashboard.getInstance().getTelemetry().addData("angleerr", AngleError);
        FtcDashboard.getInstance().getTelemetry().addData("presencia",BPresence);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls",nBalls);
    }
}
