
package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TurretController:
 * - MANUAL: Run-to-position to one of {-75, 0, +75} deg with holding power.
 * - AUTO: If Limelight sees a tag and tx exceeds a small deadband, adjust setpoint; otherwise hold.
 */
public class TurretController {
    public enum Mode { AUTO, MANUAL }

    // === MOTOR / TURRET CONFIG ===
    private static final double MOTOR_CPR = 537.6; // GoBILDA 5203 19.2:1
    private static final double TURRET_GEAR_RATIO = 5;  //??
    private static final double TICKS_PER_360 = MOTOR_CPR * TURRET_GEAR_RATIO;

    // === SOFT LIMITS (degrees) ===
    private static final double MIN_DEG = -35.0;
    private static final double MAX_DEG = 35.0;

    // === MANUAL TARGETS (degrees) ===
    private static double[] MANUAL_POSITION_DEG = { -35.0, 0.0,  35, 0.0};
    private int manualIndex = 1; // start at 0 degrees
    // Debounce for manual selection (ms)
    private static final long DEBOUNCE_MS = 150;
    private long lastManualChangeMs = 0;

    // === LIMELIGHT AIMING (camera mounted on turret) ===
    private static final double AIM_DEADBAND_DEG = 0.5;//3.0; //4.0; // "few degrees" window
    private static final double CAMERA_CROSSHAIR_YAW_OFFSET_DEG = 0.0;

    // ADDED (non-dashboard): fixed values in code; can be changed by code or setter
    private double AIM_OFFSET_DEG = 0.0;            // + = aim RIGHT of tag, - = aim LEFT of tag
    private static final double AUTO_POWER_CLAMP = 0.85; // same as your original clamp
    private static final double AUTO_KICK = 0.07;        // same as your original kick
    private double lastAimErrDeg = 0.0;                 // for telemetry/logic visibility

    // Auto fallback timeout (ms)
    private static final long AUTO_LOSS_TIMEOUT_MS = 25000;

    // === RUN_TO_POSITION behavior ===
    private static final double MOVE_POWER = 0.55; // power while moving
    private static final double HOLD_POWER = 0.12; // constant hold torque 0.12
    private static final int RTP_TOLERANCE_TICKS = 8; //12; // "close enough" window

    // If the turret is "punchy" or oscillates, Kp (Proportional) is likely too high
    // Kd (Derivative): This acts as a brake when the motor is moving too fast.
    // If the turret hits limits because it has too much momentum, a higher Kd will help it slow down as it gets closer to the goal
//  PIDController turretPID = new PIDController(-0.00001, 0, 0.0009); // starting gains - slow to move the turret
//  PIDController turretPID = new PIDController(-0.0001, 0, 0.0009); // 2nd round, still slow

    PIDController turretPID = new PIDController(-0.025, 0, 0.00001); // TUNE THESE
//    PIDController turretPID = new PIDController(-0.04, 0, 0.00001);

    // --- Hardware ---
    private DcMotorEx turret;
    private Limelight3A limelight;

    // --- State ---
//  private Mode mode = Mode.MANUAL;
    private Mode mode = Mode.AUTO;
    // Desired angle (deg) we are holding/moving to
    private double desiredDeg = MANUAL_POSITION_DEG[manualIndex];

    // Limelight data
    private long lastValidMs = 0;
    private boolean llValid = false;
    private double llTxDeg = 0.0;
    private double llTyDeg = 0.0;
    private double llTa = 0.0;

    // Names / pipeline
    private final String turretName;
    private final String limelightName;
    private final int pipeline;

    public TurretController(String turretName, String limelightName, int pipeline) {
        this.turretName = turretName;
        this.limelightName = limelightName;
        this.pipeline = pipeline;
    }

    public void turret_init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize target to current manual position
//      desiredDeg = clampDeg(MANUAL_POSITION_DEG[manualIndex]);
//      turret.setTargetPosition(degToTicks(desiredDeg));
//
//      turret.setMode(DcMotor.RunMode.RUN_TO_POSITION); // start in RTP
//      turret.setPower(HOLD_POWER);
    }

    public void limelight_init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.pipelineSwitch(pipeline);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /** Call each loop. Moves only when manual changes or AUTO is actively aiming. */
    public void update() {
        // --- Read Limelight ---
        LLResult result = limelight.getLatestResult();
        llValid = (result != null && result.isValid());
        llTxDeg = 0.0;
        llTyDeg = 0.0;
        llTa = 0.0;
        if (llValid) {
            llTxDeg = result.getTx(); // degrees left(-)/right(+)
            llTyDeg = result.getTy();
            llTa = result.getTa();
            lastValidMs = System.currentTimeMillis();
        }

        // --- Auto loss fallback ---
//      if (mode == Mode.AUTO) {
//          long sinceValid = System.currentTimeMillis() - lastValidMs;
//          if (!llValid \
//
//              sinceValid > AUTO_LOSS_TIMEOUT_MS) {
//              setModeManual(); // stop auto aiming; hold last desiredDeg
//          }
//      }

        // --- Decide desiredDeg ---
        if (mode == Mode.AUTO && llValid) {
//          if (llValid) {
            if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // ADDED (no-dashboard): compute adjusted tx and offset error
            double llTxAdj = llTxDeg + CAMERA_CROSSHAIR_YAW_OFFSET_DEG;
            double errYaw = AIM_OFFSET_DEG - llTxAdj;
            lastAimErrDeg = errYaw; // store for telemetry

            // ADDED: Drive PID toward target= AIM_OFFSET_DEG (instead of 0)
//          double autoPower = turretPID.update(0, llTxDeg);
            double autoPower = turretPID.update(AIM_OFFSET_DEG, llTxAdj);

            // Optional: Clamp power to prevent the turret from spinning too fast
//          autoPower = Math.max(-0.35, Math.min(0.35, autoPower));
            autoPower = Math.max(-AUTO_POWER_CLAMP, Math.min(AUTO_POWER_CLAMP, autoPower)); // keep your original clamp value

            // Add a small "kick" to overcome friction
//          autoPower += Math.signum(llTxDeg) * 0.05;
            autoPower += Math.signum(errYaw) * AUTO_KICK; // kick in direction that reduces error

            double currentPos = turret.getCurrentPosition();
            // Apply Soft Limits to Auto-Align
//            if (autoPower > 0 && currentPos >= degToTicks(MAX_DEG)) {
//                turret.setPower(0);
//            }
//            else if (autoPower < 0 && currentPos <= degToTicks(MIN_DEG)) {
//                turret.setPower(0);
//            }
            if (currentPos >= degToTicks(MAX_DEG)) {
                turret.setTargetPosition(degToTicks(MANUAL_POSITION_DEG[2]));
                if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                turret.setPower(HOLD_POWER);
            }
            else if (currentPos <= degToTicks(MIN_DEG)) {
                turret.setTargetPosition(degToTicks(MANUAL_POSITION_DEG[0]));
                if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                turret.setPower(HOLD_POWER);
            }
//          else if (Math.abs(llTxDeg) > AIM_DEADBAND_DEG) {
            else if (Math.abs(errYaw) > AIM_DEADBAND_DEG) { // use errYaw vs deadband
                if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                turret.setPower(autoPower);
            }
            else {
                // ADDED: hold position against outtake vibration once inside deadband
                if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                turret.setPower(0);
                //turret.setPower(HOLD_POWER);
            }
        }
        // In MANUAL, desiredDeg is set by manual selection methods
//      else{
        else if (mode == Mode.MANUAL) {
            // --- Command motor in RUN_TO_POSITION ---
            int desiredTicks = degToTicks(desiredDeg);
            turret.setTargetPosition(desiredTicks);
            if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            int currentTicks = turret.getCurrentPosition();
            int errTicks = desiredTicks - currentTicks;
            // Move if far; hold if close
            if (Math.abs(errTicks) > RTP_TOLERANCE_TICKS) {
                turret.setPower(MOVE_POWER);
            } else {
                turret.setPower(HOLD_POWER);
            }
        }
        else {
            turret.setTargetPosition(degToTicks(0));
            if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            turret.setPower(HOLD_POWER);
//          turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//          turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // --- Public driver control API ---
    /** Toggle between -75°, 0°, +75° with debounce. Call on button press (edge-detected). */
    public void cycleManualPosition() {
        if (mode != Mode.MANUAL) return; // only allow in MANUAL
        long now = System.currentTimeMillis();
        if (now - lastManualChangeMs < DEBOUNCE_MS) return;
        manualIndex = (manualIndex + 1) % MANUAL_POSITION_DEG.length;
        lastManualChangeMs = now;
        setManualTargetDeg(MANUAL_POSITION_DEG[manualIndex]);
    }

    /** Direct select (D-pad) with debounce. */
    public void selectLeftDeg() { if (mode == Mode.MANUAL) setManualIndex(0); }
    public void selectCentertDeg(){ if (mode == Mode.MANUAL) setManualIndex(1); }
    public void selectRightDeg() { if (mode == Mode.MANUAL) setManualIndex(2); }
    public void selectIdx() { if (mode == Mode.MANUAL) setManualIndex(3); }

    public void customDeg(double deg) {
        MANUAL_POSITION_DEG[3] = deg;
    }

    private void setManualIndex(int idx) {
        long now = System.currentTimeMillis();
        if (now - lastManualChangeMs < DEBOUNCE_MS) return;
        manualIndex = Math.max(0, Math.min(idx, MANUAL_POSITION_DEG.length - 1));
        lastManualChangeMs = now;
        setManualTargetDeg(MANUAL_POSITION_DEG[manualIndex]);
    }

    /** Internal: set a new manual target and command RTP immediately. */
    private void setManualTargetDeg(double degrees) {
        desiredDeg = clampDeg(degrees);
        turret.setTargetPosition(degToTicks(desiredDeg));
        // Immediately update motor target so it responds without waiting for next loop timing
        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Use move power initially; update() will switch to hold when close
        turret.setPower(MOVE_POWER);
    }

    /** Enter AUTO mode (Limelight primary). */
    public void setModeAuto() {
        mode = Mode.AUTO;
        // Stay in RUN_TO_POSITION; update() will adjust desiredDeg only when tx is outside deadband
        // No immediate movement if tx ≈ 0—will just hold current desiredDeg.
    }

    /** Enter MANUAL mode. */
    public void setModeManual() {
        mode = Mode.MANUAL;
        // Reassert current manual index as desired (no motion unless different from current)
        setManualTargetDeg(MANUAL_POSITION_DEG[manualIndex]);
    }

    // --- Telemetry getters ---
    public Mode getMode() { return mode; }
    public boolean getLlValid() { return llValid; }
    public double getLlTxDeg() { return llTxDeg; }
    public double getLlTyDeg() { return llTyDeg; }
    public double getLlTa() { return llTa; }
    public double getDesiredDeg(){ return desiredDeg; }
    public double getCurrentDeg(){ return ticksToDeg(turret.getCurrentPosition()); }
    public double getCurrentPwr(){ return turret.getPower(); }

    // ADDED: convenience/telemetry for offset aiming (no dashboard)
    public double getAimOffsetDeg(){ return AIM_OFFSET_DEG; }
    public double getAimErrorDeg(){ return lastAimErrDeg; }
    public void setAimOffsetDeg(double deg){ AIM_OFFSET_DEG = deg; } // call this from auto to change offset

    // --- Helpers ---
    private double clampDeg(double deg) {
        return Range.clip(deg, MIN_DEG, MAX_DEG);
    }
    private static int degToTicks(double degrees) {
        return (int)Math.round((degrees / 360.0) * TICKS_PER_360);
    }
    private static double ticksToDeg(int ticks) {
        return (ticks / TICKS_PER_360) * 360.0;
    }

    public class PIDController {
        private double Kp, Ki, Kd;
        private double lastError = 0;
        private double integralSum = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        // ADDED: method available even if not used for dashboard
        public void setGains(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double update(double target, double current) {
            double error = target - current;
            double deltaTime = timer.seconds();
            // Calculate integral and derivative
            integralSum += (error * deltaTime);
            double derivative = (error - lastError) / deltaTime;
            lastError = error;
            timer.reset();
            return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
        }
    }
}