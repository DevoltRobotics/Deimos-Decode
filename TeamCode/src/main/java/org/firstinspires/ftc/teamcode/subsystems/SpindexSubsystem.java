package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class SpindexSubsystem extends SubsystemBase {

    HookSubsystem hookSubsystem;

    public int lastFlickSeen = Integer.MIN_VALUE;

    // ----------------- Actuador -----------------
    public CRServo spindex;

    // ----------------- Sensores -----------------
    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    public AnalogInput SpindexPos; // 0..3.3V -> 0..2π rad

    // ----------------- PID (radianes) -----------------
    public PIDFController SpindexPID;


    // Tunings en RAD (ajústalos en dashboard)
    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(
            0.13,   // P
            0.00,   // I
            0.012,  // D
            0.00    // F
    );

    // Estabilidad / antifricción
    public static double K_STATIC = 0.0;                 // empujón anti-fricción
    public static double ERROR_DEADBAND_RAD = 0.05;      // ~ 4.58°
    public static double MIN_SERVO = 0.05;
    public static double MAX_SERVO = 1.0;

    // Slew del setpoint para evitar picos
    public static double TARGET_SLEW_RAD_PER_SEC = 8.0;  // ~458°/s
    public static double POS_LPF_ALPHA = 0.25;           // filtro lectura

    // ----------------- Lógica automática (distancia) -----------------
    public static double ENTER_DISTANCE_CM = 2.5;             // umbral entrar
    public static double EXIT_DISTANCE_CM  = 3.5;             // umbral salir (histeresis)
    public static double TRIGGER_COOLDOWN_MS = 250;           // anti rebote tiempo
    public static double MIN_ADVANCE_RAD = Math.toRadians(110); // avance mínimo para rearmar

    private boolean objectLatched = false;
    private double lastTriggerTimeMs = -9999;
    private double lastTriggerPosRad = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // ----------------- Objetivos (en radianes) -----------------
    // NOTA: todo interno en radianes.
    public double offsetRad =   0.6073;         // home
    public double FPosRad   = offsetRad + Math.toRadians(60) ;             // posición de "ready to shoot"


    private double targetPosRad = 0.0;   // objetivo "crudo" (lo que pide la lógica)
    private double targetCmdRad = 0.0;   // objetivo suavizado que persigue el PID

    // Estado
    public boolean Shootmode = true;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    private int nBalls = -1;             // contador interno

    // Estado de medición (radianes)
    private double posFilteredRad = 0.0;
    private double lastUpdateTimeS = 0.0;

    // Color
    private DetectedColor currentColorL = DetectedColor.UNKNOWN;
    float normRed, normGreen, normBlue;
    NormalizedRGBA colorsL;

    // Telemetría
    public double rawSpindexP; // posición actual en rad (raw normalizada)
    public double Spindexp; // posición actual en rad (raw normalizada)

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    public SpindexSubsystem(HardwareMap hMap,HookSubsystem hookSubsystem) {
        this.hookSubsystem = hookSubsystem;

        spindex = hMap.get(CRServo.class, "spindex");

        SpindexPos = hMap.get(AnalogInput.class, "Spencoder");

        colorSensorL = hMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hMap.get(NormalizedColorSensor.class, "colorR");
        colorSensorL.setGain(8);

        SpindexPID = new PIDFController(SPcoeffs);

        // Inicializar estados con la lectura actual
        double pos0 = normalizeRadians((SpindexPos.getVoltage() / 3.3) * 2 * Math.PI);
        posFilteredRad = pos0;
        targetCmdRad   = pos0;
        targetPosRad   = pos0;
        lastUpdateTimeS = timer.seconds();

        lastFlickSeen = hookSubsystem.nFlick;

    }

    // ----------------- Utilidades de ángulo (rad) -----------------

    /** Diferencia circular envuelta a (-π, π] */
    private static double deltaRad(double to, double from) {
        return normalizeRadians(to - from);
    }

    /** Distancia por el arco corto (valor absoluto) */
    private static double circularAbsRad(double a, double b) {
        return Math.abs(deltaRad(a, b));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ----------------- Periodic -----------------
    @Override
    public void periodic() {
        // ===== Lecturas base =====
        rawSpindexP = normalizeRadians((SpindexPos.getVoltage() / 3.3) * 2.0 * Math.PI);

        Spindexp = normalizeRadians(rawSpindexP - offsetRad);

        // LPF sobre la medición (en el delta envuelto)
        double dPos = deltaRad(Spindexp, posFilteredRad);
        posFilteredRad = normalizeRadians(posFilteredRad + POS_LPF_ALPHA * dPos);

        // ===== Lógica automática con booleano Shootmode =====
        // Cuando Shootmode == false, indexa automáticamente hasta 3 pelotas.
        // Cuando completa, regresa a Shootmode = true, nBalls = -1, y mueve a FPosRad.
        double distCm = ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM);
        double nowMs = timer.milliseconds();

        if (!Shootmode) {
            if (!approximatelyEqual(targetPosRad, offsetRad) && nBalls == -1) {
                // primer paso: volver a offset y arrancar contador
                targetPosRad = 0;
                nBalls = 0;
            } else if (nBalls >= 0 && nBalls < 3) {
                // Trigger por flanco: presencia + cooldown
                if (!objectLatched
                        && distCm <= ENTER_DISTANCE_CM
                        && (nowMs - lastTriggerTimeMs) >= TRIGGER_COOLDOWN_MS) {

                    nBalls += 1;

                    // Avanza plato SOLO si aún no llenamos las 3
                    if (nBalls < 3) {
                        targetPosRad = normalizeRadians(targetPosRad + Math.toRadians(120)); // +120°
                    }

                    objectLatched = true;
                    lastTriggerTimeMs = nowMs;
                    lastTriggerPosRad = Spindexp;
                }

                // soltar el latch cuando: salió del umbral y avanzamos suficiente
                if (objectLatched) {
                    boolean movedEnough = circularAbsRad(Spindexp, lastTriggerPosRad) >= MIN_ADVANCE_RAD;
                    if (distCm >= EXIT_DISTANCE_CM && movedEnough) {
                        objectLatched = false;
                    }
                }
            } else if (nBalls == 3) {
                // terminado: pasar a modo tiro y mandar a FPos
                Shootmode = true;
                nBalls = -1;
                targetPosRad += Math.toRadians(60);
            }
        }

            if ((hookSubsystem.nFlick == 3) ){
                setShootMode(false);
                hookSubsystem.nFlick = 0;
            }




        // ===== Control (error-a-cero + slew + deadband + K_STATIC) =====
        SpindexPID.setMinimumOutput(MIN_SERVO);

        // Slew del objetivo "real"
        double nowS = timer.seconds();
        double dt = Math.max(0.001, nowS - lastUpdateTimeS);
        lastUpdateTimeS = nowS;

        double maxStep = TARGET_SLEW_RAD_PER_SEC * dt;
        double errToTarget = deltaRad(targetPosRad, targetCmdRad);
        double step = clamp(errToTarget, -maxStep, maxStep);
        targetCmdRad = normalizeRadians(targetCmdRad + step);

        // Error circular entre targetCmd y posición filtrada
        double errorRad = deltaRad(targetCmdRad, posFilteredRad);

        // PID "error-a-cero": setpoint=0, medición=error
        SpindexPID.setCoefficients(SPcoeffs);
        SpindexPID.setSetPoint(0.0);
        double power = SpindexPID.calculate(errorRad);

        if (Math.abs(errorRad) > ERROR_DEADBAND_RAD) {
            power += K_STATIC * Math.signum(errorRad);
        } else {
            power = 0.0; // quieto en ventana muerta
        }

        if (Double.isNaN(power)) power = 0.0;
        power = clamp(power, -MAX_SERVO, MAX_SERVO);

        spindex.setPower(power);

        // ===== Colores =====
        colorsL = colorSensorL.getNormalizedColors();
        normRed   = colorsL.red   / colorsL.alpha;
        normGreen = colorsL.green / colorsL.alpha;
        normBlue  = colorsL.blue  / colorsL.alpha;

        if (normRed > 0.35 && normBlue > 0.35 && normGreen < 0.35) {
            currentColorL = DetectedColor.PURPLE;
        } else if (normGreen > 0.35 && normBlue < 0.35 && normRed < 0.35) {
            currentColorL = DetectedColor.GREEN;
        } else {
            currentColorL = DetectedColor.UNKNOWN;
        }

        // ===== Telemetría =====
        FtcDashboard.getInstance().getTelemetry().addData("Shootmode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
        FtcDashboard.getInstance().getTelemetry().addData("posRaw(rad)", Spindexp);
        FtcDashboard.getInstance().getTelemetry().addData("posFilt(rad)", posFilteredRad);
        FtcDashboard.getInstance().getTelemetry().addData("targetRaw(rad)", targetPosRad);
        FtcDashboard.getInstance().getTelemetry().addData("targetCmd(rad)", targetCmdRad);
        FtcDashboard.getInstance().getTelemetry().addData("error(rad)", errorRad);
        FtcDashboard.getInstance().getTelemetry().addData("power", power);
        FtcDashboard.getInstance().getTelemetry().addData("Distance(cm)", "%.3f", distCm);
        FtcDashboard.getInstance().getTelemetry().addData("Shootmode",Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("Flicks",hookSubsystem.nFlick);

    }

    // ----------------- API pública -----------------

    /** Cambia a modo automático (false) o manual/tiro (true) */
    public void setShootMode(boolean shoot) {
        if (this.Shootmode != shoot) {
            this.Shootmode = shoot;
            // reset de estado de indexado al entrar a auto
            if (!shoot) {
                objectLatched = false;
                lastTriggerTimeMs = -9999;
                lastTriggerPosRad = Spindexp;
                nBalls = -1;
            }
        }
    }

    public boolean isShootMode() {
        return Shootmode;
    }

    /** Fija objetivo en radianes (manual) */
    public void setTargetPos(double targetPosRad) {
        this.targetPosRad = normalizeRadians(targetPosRad);
    }

    /** Ir al home/offset */
    public void goOffset() {
        this.targetPosRad = offsetRad;
    }

    /** Ir a FPos (ready) */
    public void goFPos() {
        this.targetPosRad = FPosRad;
    }

    /** Avanza/retrocede 120° (2π/3 rad) manualmente */
    public void nextPos(boolean reverse) {
        double step = 2.0 * Math.PI / 3.0;
        this.targetPosRad = normalizeRadians(this.targetPosRad + (reverse ? step : -step));
    }

    /** Set/Get contador por si quieres manipular desde fuera */
    public void setNBalls(int n) {
        this.nBalls = n;
    }
    public int  getNBalls()      { return nBalls; }

    /** Getters útiles */
    public double getCurrentRad() { return Spindexp; }
    public double getTargetRad()  { return targetPosRad; }

    /** Helpers en grados por si los ocupas */
    public void setTargetDeg(double deg) { setTargetPos(Math.toRadians(deg)); }
    public double getCurrentDeg() { return Math.toDegrees(getCurrentRad()); }
    public double getTargetDeg()  { return Math.toDegrees(getTargetRad()); }

    // ----------------- Util -----------------
    private static boolean approximatelyEqual(double a, double b) {
        return Math.abs(deltaRad(a, b)) < 1e-3; // ~0.057°
    }
}
