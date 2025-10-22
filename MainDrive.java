package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {

///////////////////////////////////////////////////////

    // ----------------- HARDWARE -----------------
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor collect_motor;
    private DcMotor basket_motor;
    private DcMotor lifter_motor; 
    private CRServo back_door; 
    private Servo left_gripper;
    private Servo right_gripper;

    // ----------------- SENSORS -----------------
    private DigitalChannel door_sensor;
    private DigitalChannel basket_sensor;

    // ----------------- DRIVE -----------------
    double drive_speed_factor = 1.0;
    double speed_factor_change = 0.25;
    double turn_speed_factor = 0.75;

    // ----------------- COLLECTOR -----------------
    boolean collect_active = false;
    double collect_power_level = 1.0;

    // ----------------- BASKET -----------------
    boolean basket_retracted;
    double basket_power_level = 1.0;
    int basket_turns = 4;   
    int step_ticks = 144;   

    // ----------------- BACK DOOR -----------------
    private boolean last_triangle = false;
    private boolean back_door_running = false;
    private double last_direction = 1.0; 
    private ElapsedTime back_door_timer = new ElapsedTime();
    private final double BACK_DOOR_DURATION = 1.75; 

    // ----------------- GRIPPERS -----------------
    private double gripper_open = 0.0;
    private double gripper_closed = 1.0;

    // ----------------- LIFTER -----------------
    private boolean lifter_running = false;
    private double lifter_power = 1.0;
    private double lifter_current_power = 0;
    private String lifter_status = "STOPPED"; 
    private double lifter_hold_ratio = 0.5;

    // Trigger debounce booleans
    private boolean last_lt = false;
    private boolean last_rt = false;

    // ----------------- BUTTON STATES -----------------
    boolean last_a = false;
    boolean last_b = false;
    boolean last_square = false;
    boolean last_home = false;

///////////////////////////////////////////////////////

    @Override
    public void runOpMode() {
        initHardware();
        initLifterMotor();
        initBasket();
        initServo();
        
        lifter_motor.setPower(0);
        lifter_status = "STOPPED";

        waitForStart();
        checkDoorSensorAtStart();

        while (opModeIsActive()) {
            handleDrive();
            handleDriveSpeedAdjust();
            handleCollector();
            handleBasket();
            handleBackDoor(gamepad1.triangle);
            handleGrippers();
            handleClimber(); 
            handleClimbSequenceActivation(); 
            updateTelemetry();
        }
    }


    // -------------------  Climb Sequence -------------------
    private void handleClimbSequenceActivation() {
        if (gamepad1.share && !last_home) { 
            climbingSequence();
        }
        last_home = gamepad1.share;
    }

    private void climbingSequence() {
        telemetry.addLine("CLIMBING SEQUENCE STARTED");
        telemetry.update();

     if (!basket_retracted) {
            telemetry.addLine("Retracting basket...");
            telemetry.update();
            activateBasket(-basket_turns); // retract
            while (opModeIsActive() && basket_motor.isBusy()) {
                idle();
            }
        }
        
        if (collect_active || Math.abs(collect_motor.getPower()) > 0.05) {
            telemetry.addLine("Stopping collector...");
            telemetry.update();
            collect_motor.setPower(0);
            collect_active = false;
        }

        if (!door_sensor.getState()) {
            telemetry.addLine("Opening back door...");
            telemetry.update();
            last_direction = 1.0;
            back_door.setPower(last_direction);
            back_door_timer.reset();
            while (opModeIsActive() && back_door_timer.seconds() < BACK_DOOR_DURATION) {
                idle();
            }
            back_door.setPower(0);
            
            
                 
        if (!basket_retracted) {
            telemetry.addLine("Retracting basket...");
            telemetry.update();
            activateBasket(-basket_turns); // retract
            while (opModeIsActive() && basket_motor.isBusy()) {
                idle();
            }
        }
        }

        // Close grippers
        telemetry.addLine("Closing grippers...");
        telemetry.update();
        left_gripper.setPosition(gripper_open);
        right_gripper.setPosition(gripper_closed);
        sleep(500);

        //  Start lifter (UP direction)
        telemetry.addLine("Starting lifter...");
        telemetry.update();
        lifter_motor.setPower(-lifter_power);
        lifter_running = true;
        lifter_current_power = -lifter_power;
        lifter_status = "UP";
    }


//----------------Hardware init----------------

    private void initHardware() {
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        collect_motor = hardwareMap.get(DcMotor.class, "collector");
        basket_motor = hardwareMap.get(DcMotor.class, "basket");
        lifter_motor = hardwareMap.get(DcMotor.class, "lifter_motor");
        back_door = hardwareMap.get(CRServo.class, "back_door");
        left_gripper = hardwareMap.get(Servo.class, "left_gripper");
        right_gripper = hardwareMap.get(Servo.class, "right_gripper");

        door_sensor = hardwareMap.get(DigitalChannel.class, "door_sensor");
        door_sensor.setMode(DigitalChannel.Mode.INPUT);

        basket_sensor = hardwareMap.get(DigitalChannel.class, "basket_sensor");
        basket_sensor.setMode(DigitalChannel.Mode.INPUT);

        right_drive.setDirection(DcMotor.Direction.REVERSE);
    }
    
    private void initLifterMotor() {
    //Motor setup
    lifter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    telemetry.addLine("Moving lifter to 0 position...");
    telemetry.update();

    //Move toward 0
    lifter_motor.setTargetPosition(0);
    lifter_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lifter_motor.setPower(lifter_power); 

    //While loop runs during init
    
    while (lifter_motor.isBusy()) {
        telemetry.addData("Lifter Position", lifter_motor.getCurrentPosition());
        telemetry.update();
    }
    
    

    //Stop and release power
    lifter_motor.setPower(0);
    lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    telemetry.addLine("Lifter is at 0 position. Power removed.");
    telemetry.update();
    }

    private void initBasket() {
        basket_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basket_motor.setTargetPosition(0);
        basket_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        basket_retracted = !basket_sensor.getState();
    }
    
    private void initServo() {
        back_door.setPower(0);
        left_gripper.setPosition(gripper_open);
        right_gripper.setPosition(gripper_closed);
    }

//----------------Check sensors----------------

    private void checkDoorSensorAtStart() {
       
        if (!door_sensor.getState() && last_direction > 0) {
            last_direction *= -1;
        }
    }
    

//----------------Handle functions----------------    
    private void handleDrive() {
        double left_power = (-gamepad1.left_stick_y - (gamepad1.right_stick_x * turn_speed_factor)) * drive_speed_factor;
        double right_power = (-gamepad1.left_stick_y + (gamepad1.right_stick_x * turn_speed_factor)) * drive_speed_factor;
        left_drive.setPower(left_power);
        right_drive.setPower(right_power);
    }

    private void handleDriveSpeedAdjust() {
        if (gamepad1.dpad_up||gamepad1.dpad_up && drive_speed_factor < 1.0) {
            drive_speed_factor += speed_factor_change;
            sleep(200);
        }
        if (gamepad1.dpad_down ||gamepad1.dpad_down  && drive_speed_factor > speed_factor_change) {
            drive_speed_factor -= speed_factor_change;
            sleep(200);
        }
    }

    private void handleCollector() {
        if (gamepad1.a && !last_a) toggleCollector(false);
        last_a = gamepad1.a;

        if (gamepad1.b && !last_b) toggleCollector(true);
        last_b = gamepad1.b;
    }

    private void handleBasket() {
        basket_retracted = !basket_sensor.getState();

        if (basket_retracted && basket_motor.getPower() < 0) {
            basket_motor.setPower(0);
            basket_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.square && !last_square) {
            if (basket_retracted) activateBasket(basket_turns);
            else 
            {
                activateBasket(-basket_turns);
            }
        }
        last_square = gamepad1.square;

        if (!basket_motor.isBusy() && basket_motor.getPower() != 0) {
            basket_motor.setPower(0);
            basket_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void handleGrippers(){

     boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

     // Open on D-Pad left
        if (dpadLeft) {
        left_gripper.setPosition(gripper_open);
        right_gripper.setPosition(gripper_closed); // mirrored
     }

     // Close on D-Pad right
     if (dpadRight) {
        left_gripper.setPosition(gripper_closed);
        right_gripper.setPosition(gripper_open); // mirrored
     }
    

    
  
    }

    private void handleBackDoor(boolean triangle_pressed) {
        boolean sensor_triggered = !door_sensor.getState();
        if (triangle_pressed && !last_triangle) {
            if (!back_door_running) {
                double next_direction = last_direction * -1;
                if (next_direction <= 0 && sensor_triggered) {
                } else {
                    last_direction = next_direction;
                    back_door.setPower(last_direction);
                    back_door_timer.reset();
                    back_door_running = true;
                }
            }
        }
        
        last_triangle = triangle_pressed;

        if (back_door_running && last_direction < 0 && sensor_triggered) {
            back_door.setPower(0);
            back_door_running = false;
        }
        if (back_door_running && last_direction > 0 && back_door_timer.seconds() >= BACK_DOOR_DURATION) {
            back_door.setPower(0);
            back_door_running = false;
        }
    }
     
    private void handleClimber() {
        double up_trigger = gamepad1.left_trigger;
        double down_trigger = gamepad1.right_trigger;

        // LEFT TRIGGER (UP)
        if (up_trigger > 0.1 && !last_lt) {
            if (!lifter_running) {
                lifter_motor.setPower(lifter_power);
                lifter_running = true;
                lifter_current_power = lifter_power;
                lifter_status = "UP";
            } else {
                lifter_motor.setPower(lifter_power * lifter_hold_ratio);
                lifter_running = false;
                lifter_current_power = lifter_power * lifter_hold_ratio;
                lifter_status = "HOLD (UP)";
            }
            last_lt = true;
        } else if (up_trigger < 0.1) {
            last_lt = false;
        }

        // RIGHT TRIGGER (DOWN)
        if (down_trigger > 0.1 && !last_rt) {
            if (!lifter_running) {
                lifter_motor.setPower(-lifter_power);
                lifter_running = true;
                lifter_current_power = -lifter_power;
                lifter_status = "DOWN";
            } else {
                lifter_motor.setPower(-lifter_power * lifter_hold_ratio);
                lifter_running = false;
                lifter_current_power = -lifter_power * lifter_hold_ratio;
                lifter_status = "HOLD (DOWN)";
            }
            last_rt = true;
        } else if (down_trigger < 0.1) {
            last_rt = false;
        }
    }


//----------------Extra functions----------------   
    private void activateBasket(int steps) {
        if (steps < 0 && basket_retracted) return;
        int target = basket_motor.getCurrentPosition() + (steps * 144);
        basket_motor.setTargetPosition(target);
        basket_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        basket_motor.setPower(basket_power_level);
    }

    private void toggleCollector(boolean reverse) {
        collect_active = !collect_active;
        double power = reverse ? -collect_power_level : collect_power_level;
        collect_motor.setPower(collect_active ? power : 0);
    }    

    private void updateTelemetry() {
        boolean door_triggered = !door_sensor.getState();

        telemetry.addData("Drive L", left_drive.getPower());
        telemetry.addData("Drive R", right_drive.getPower());
        telemetry.addData("Speed", drive_speed_factor);
        telemetry.addData("Collector", collect_motor.getPower());
        telemetry.addData("Basket Pos", basket_motor.getCurrentPosition());
        telemetry.addData("Basket Retracted", basket_retracted);
        telemetry.addData("Back Door Dir", back_door_running ? last_direction : 0);
        telemetry.addData("Door Sensor Raw", door_sensor.getState());
        telemetry.addData("Door Triggered", door_triggered);
        telemetry.addData("Door Status", door_triggered ? "CLOSED" : "OPEN");
        telemetry.addData("Lifter Status", lifter_status);
        telemetry.addData("Lifter Power", lifter_current_power);
        telemetry.update();
    }
}