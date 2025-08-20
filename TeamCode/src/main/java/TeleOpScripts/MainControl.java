package TeleOpScripts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Teleop", group = "Teleop")
public class MainControl extends OpMode {

    // Motor variables
    private DcMotor rightFront;   // RF
    private DcMotor rightBack;    // RB
    private DcMotor leftFront;    // LF
    private DcMotor leftBack;     // LB

    // FTC Dashboard instance
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Dashboard configurable variables - these can be tuned live on the dashboard
    public static double SPEED_MULTIPLIER = 1.0;           // Overall speed multiplier (0.1 to 1.0). Use smaller number for slower speed.
    public static double DEADZONE = 0.1;                   // Joystick deadzone threshold
    public static boolean FIELD_CENTRIC = false;          // Enable field-centric drive (leave off)

    // Variables for telemetry
    private boolean joystickActive = false;
    private double strafe = 0;
    private double forward = 0;
    private double rotate = 0;
    private double leftBackPower = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    @Override
    public void init() {
        // Initialize motors - CHANGE THESE NAMES TO MATCH YOUR ROBOT CONFIGURATION
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake when power is zero
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - Dashboard Connected");
    }

    @Override
    public void loop() {
        // Game loop
        mecanumMovement();


        // Handle all telemetry
        updateTelemetry();
    }

    // Other functions here
    private void mecanumMovement() {
        // Get joystick values based on configuration
        strafe = gamepad1.left_stick_x;      // Left/right movement
        rotate = gamepad1.right_stick_x;     // Rotation

        // Forward/backward using both joysticks
        // Choose higher Y value for forward/backward movement
        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;
        forward = Math.max(leftY, rightY);

        // Check if any joystick is being used (outside deadzone)
        joystickActive = Math.abs(strafe) > DEADZONE ||
                Math.abs(forward) > DEADZONE ||
                Math.abs(rotate) > DEADZONE;

        if (joystickActive) {
            // Calculate motor powers using cleaned up mecanum drive formula
            leftBackPower = (forward - strafe + rotate) * SPEED_MULTIPLIER;
            leftFrontPower = (forward + strafe - rotate) * SPEED_MULTIPLIER;
            rightFrontPower = (forward + strafe - rotate) * SPEED_MULTIPLIER;
            rightBackPower = (forward - strafe + rotate) * SPEED_MULTIPLIER;

            // Clip powers to valid range
            leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
            leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
            rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
            rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);

            // Set motor powers
            leftBack.setPower(leftBackPower);
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);
        } else {
            // No joystick input - stop all motors
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftBackPower = 0;
            leftFrontPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
        }
    }

    private void updateTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();

        // Dashboard telemetry
        packet.put("Joystick Active", joystickActive);
        packet.put("Strafe", strafe);
        packet.put("Forward", forward);
        packet.put("Rotate", rotate);
        packet.put("Speed Multiplier", SPEED_MULTIPLIER);
        packet.put("Deadzone", DEADZONE);

        if (joystickActive) {
            packet.put("Motor Powers", "Active");
            packet.put("LB Power", leftBackPower);
            packet.put("LF Power", leftFrontPower);
            packet.put("RF Power", rightFrontPower);
            packet.put("RB Power", rightBackPower);
        } else {
            packet.put("Motor Powers", "Stopped");
        }

        // Send telemetry to dashboard
        dashboard.sendTelemetryPacket(packet);

        // Driver station telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Joystick Active", joystickActive);
        telemetry.addData("Speed Multiplier", SPEED_MULTIPLIER);
        telemetry.update();
    }
}