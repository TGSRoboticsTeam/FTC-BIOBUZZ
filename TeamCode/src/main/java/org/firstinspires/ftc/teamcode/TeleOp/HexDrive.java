package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
// IMU is no longer strictly needed for drive, but kept for initialization if mechanisms use it.
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//@Disabled
@TeleOp(name = "HexDrive", group = "Swerve")
//@Disabled
public class HexDrive extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    private DcMotor intake1, intake2;
    private DcMotor launcher1, launcher2;
    private Servo blocker;

    private IMU imu; // Kept IMU object, but drive logic ignores its data
    private VoltageSensor voltageSensor;


    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. CRITICAL: OFFSETS (Using your measured values) ---
    final double FRONT_LEFT_OFFSET  = 0.05;
    final double FRONT_RIGHT_OFFSET = 1.39;
    final double BACK_LEFT_OFFSET   = 3.16;
    final double BACK_RIGHT_OFFSET  = 0.82;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double ADJUSTER_DEADBAND = 0.05; // Deadband for G2 stick control

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;



    // --- 9. TOGGLE STATE VARIABLES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;


    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;



    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        imu.resetYaw();

        // No headingOffset is needed for Robot-Centric drive
        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;


        while (opModeIsActive()) {

            // --- Toggle Logic for Calibration Mode (Gamepad 1) ---
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // Speed Limiter Logic (Gamepad 1)
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // --- NO YAW RESET LOGIC FOR ROBOT-CENTRIC ---

            // --- CALIBRATION MODE CHECK ---
            if (isCalibrationModeActive) {

                runCalibrationMode(
                        new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive},
                        new CRServo[]{frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer}
                );
                continue;
            }


            // ------ IMU RESETTING ----- //
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            // ------ DRIVE INPUTS (FIELD-CENTRIC) ------ //
            // Joystick inputs are the final robot-centric inputs
            double rawY = -gamepad1.left_stick_y * speedMultiplier; // Forward/Backward
            double rawX = gamepad1.left_stick_x * speedMultiplier;  // Strafe Left/Right
            double rot = gamepad1.right_stick_x * speedMultiplier;     // Rotation

            // Code for field centric
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            /* // !!!! TRY THIS OUT INSTEAD, NOT THIS, THE UNCOMMENTED STUFF BELOW IT !!!! //
            double robotX = rawX * Math.cos(-botHeading) - rawY * Math.sin(-botHeading);
            double robotY = rawX * Math.sin(-botHeading) + rawY * Math.cos(-botHeading);
            //*/
            double robotX = rawX * Math.cos(botHeading) + rawY * Math.sin(botHeading);
            double robotY = -rawX * Math.sin(botHeading) + rawY * Math.cos(botHeading);

            // Swerve Kinematics (Unchanged)
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            // Calculate wheel speeds and normalize
            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);
            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            // Steering Logic (Unchanged)
            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
                framesSinceLastMoved += 1;
            }

            // Lock wheels override ('X' formation)
            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR = Math.PI / 4;
                targetAngleBL = Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply swerve module outputs
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, -speedFrontRight, targetAngleFR); // !!! POTENTIALLY CHANGE !!! //
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, -speedBackRight, targetAngleBR);

        }
    }

    // --- HELPER METHODS ---

    private void initializeHardware() {
        // --- Swerve Drive Hardware ---
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive"); // Motor Port 3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); // Motor Port 2
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive"); // Motor Port 1
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive"); // Motor Port 0
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer"); // Servo Port 0
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer"); // Servo Port 1
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer"); // Servo Port 5
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer"); // Servo Port 4
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder"); // Analog Input Devices Port 0
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder"); // Analog Input Devices Port 1
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder"); // Analog Input Devices Port 3
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder"); // Analog Input Devices Port 2

        intake1  = hardwareMap.get(DcMotor.class, "intake_1"); // Expansion Motor Port 0
        intake2 = hardwareMap.get(DcMotor.class, "intake_2"); // Expansion Motor Port 1

        launcher1   = hardwareMap.get(DcMotor.class, "launcher_1"); // Expansion Motor Port 2
        launcher2  = hardwareMap.get(DcMotor.class, "launcher_2"); // Expansion Motor Port 3
        blocker = hardwareMap.get(Servo.class, "blocker"); // Expansion Servo Port 0

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");

        // CRITICAL: IMU INITIALIZATION IS STILL NEEDED FOR HUB TO START
        // The values here don't matter for the drive function since we ignore the heading.
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // !!!! CHANGE TO .RIGHT, then if that doesn't work, change to .RIGHT !!!! //
                )
        );
        imu.initialize(parameters);

        // --- Swerve Drive Motor Direction Fix ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // !!!! POTENTIALLY REMOVE THE SPEED INVERSIONS AND SET THE RIGHTS TO .FORWARD !!!! //
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        // Swerve module control logic (unchanged)
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Steering Fix: Invert servo power to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private void runCalibrationMode(DcMotor[] driveMotors, CRServo[] steerServos) {
        for (DcMotor motor : driveMotors) { motor.setPower(0); }
        for (CRServo servo : steerServos) { servo.setPower(0); }

        telemetry.addData("Mode", "**CALIBRATION - PID DISABLED**");
        telemetry.addData("FL Raw Angle", getRawAngle(frontLeftEncoder));
        telemetry.addData("FR Raw Angle", getRawAngle(frontRightEncoder));
        telemetry.addData("BL Raw Angle", getRawAngle(backLeftEncoder));
        telemetry.addData("BR Raw Angle", getRawAngle(backRightEncoder));
        telemetry.addData("Exit", "Press Right Stick Button (R3) to EXIT.");
        telemetry.update();
    }
}