package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous with FSM(test)", group="Robot")
public class Autonomous extends OpMode {
    //LynxModule controlHub;
    private double StartStatePutTime;
    private Servo down_clip_head;//down_clip_head控制夹子的旋转
    private Servo down_clip_hand;//down_clip_hand控制夹子的抓放
    private Servo down_clip_arm;//舵机夹子,顶部的那个,目前还没装好
    private Servo top_clip_head;
    private Servo top_clip_hand;
    private Servo top_clip_arm;
    private DcMotor front_left;// 四个底盘电机
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private DcMotor lift;//抬升电机
    private DcMotor intake;
    private IMU imu;// 惯性测量单元
    private boolean DCstate = true, LB_last_pressed = false; // false for open; true for close
    private boolean TCstate = true, RB_last_pressed = false; // false for open; true for close


    private final ElapsedTime runtime = new ElapsedTime();

    private Values values = new Values();

    public void init_robot(){
        //controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        //controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        telemetry.setAutoClear(false);
        //Initialize motors
        {
            down_clip_arm = hardwareMap.get(Servo.class,"DownClipArm");
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            down_clip_hand.setPosition(values.clipPositions.get("DC_close"));
            top_clip_arm = hardwareMap.get(Servo.class,"TopClipArm");
            top_clip_hand = hardwareMap.get(Servo.class,"TopClipHand");
            top_clip_head = hardwareMap.get(Servo.class,"TopClipHead");
            top_clip_hand.setPosition(values.clipPositions.get("TC_close"));
            //抬升电机
            lift = hardwareMap.get(DcMotor.class,"Lift");
            intake = hardwareMap.get(DcMotor.class,"Intake");
            // 从硬件映射中获取四个底盘电机
            front_left = hardwareMap.get(DcMotor.class, "frontLeft");
            front_right = hardwareMap.get(DcMotor.class, "frontRight");
            rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
            rear_right = hardwareMap.get(DcMotor.class, "rearRight");
            // 设置电机的转动方向
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
            rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
            // 设置电机在功率为零时的行为为制动
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //Initialize IMU & reset yaw
        {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // 如果没有这个，REV Hub 的方向将被假定为标志朝上 / USB 朝前
            imu.initialize(parameters);
            imu.resetYaw();
        }
    }

    /*public void telemetryAddData(String caption, Object value){
        telemetry.addData(caption, value);
    }
    public void telemetryUpdate(){
        telemetry.update();
    }*/

    /*private class PID_control{
        double k_p, k_i, k_d, maxi, maxoutput;
        ElapsedTime start_timer;
        double previous_time = 0.0, previous_error = 0.0, output;
        double i = 0.0;

        public PID_control(double kp, double ki, double kd, double max_i, double max_output){
            k_p = kp;
            k_i = ki;
            k_d = kd;
            maxi = max_i;
            maxoutput = max_output;
        }

        public void initTimer(){
            start_timer = new ElapsedTime();
            previous_time = 0.0;
            previous_error = 0.0;
            output = 0.0;
            i = 0.0;
        }

        public double calc(double current_position, double desire_position){
            double current_time = start_timer.seconds();
            double current_error = desire_position-current_position;

            double p = k_p * current_error;
            i += k_i * (current_error * (current_time - previous_time));
            if(i > maxi){
                i = maxi;
            }
            else if(i < -maxi){
                i = -maxi;
            }
            double d = k_d * (current_error - previous_error) / (current_time - previous_time);
            previous_error = current_error;
            previous_time = current_time;
            output = p + i + d;
            if(output > maxoutput){
                output = maxoutput;
            }
            else if(output < -maxoutput){
                output = -maxoutput;
            }
            return output;
        }
    }
    double turn_k_p = 0.001, turn_k_i = 0.01, turn_k_d = 0.005, turn_maxi = 0.5, turn_maxoutput = 1.0;

    public void leftTurn(double degree, double seconds){
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","（←)Turning Left...");
        telemetry.update();
        double origDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID_control pid = new PID_control(turn_k_p, turn_k_i, turn_k_d, turn_maxi, turn_maxoutput);
        pid.initTimer();
        while (timer.seconds() <= seconds) {
            //controlHub.clearBulkCache();  //No need
            double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //TurnCounterclockwise(pid.calc((currentDegree-origDegree), degree));
            TurnCounterclockwise(pid.calc(currentDegree, (origDegree + degree)));
        }
        stopChassis();
    }
    public void rightTurn(double degree, double seconds){
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","（→)Turning Right...");
        telemetry.update();
        double origDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID_control pid = new PID_control(turn_k_p, turn_k_i, turn_k_d, turn_maxi, turn_maxoutput);
        pid.initTimer();
        while (timer.seconds() <= seconds) {
            //controlHub.clearBulkCache();  //No need
            double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            TurnClockwise(pid.calc((origDegree-currentDegree), degree));
        }
        stopChassis();
    }
    private void TurnCounterclockwise(double power){
            //if((power < 0.07) && (power >= 0.0)){
            //    power = 0.0;
            //}
        setChassisPower(power, -power, power, -power);
    }
    private void TurnClockwise(double power){
            //if((power < 0.07) && (power >= 0.0)){
            //    power = 0.0;
            //}
        setChassisPower(-power, power, -power, power);
    }
    */


    private enum AutoState {
        START0,
        FORWARD_LIFT1,
        PUT2,
        RETRACT3,
        WAIT4,
        END5,
    };
    private AutoState autoState = AutoState.START0;

    private enum ChassisState {
        STOP,
        FORWARD,
        BACKWARD,
        LEFTWARD,
        RIGHTWARD
    };
    private ChassisState chassisState = ChassisState.STOP;
    private double chassisStartTime = 0.0, chassisSetTime = 0.0, chassisPower = 0.0;

    //loopMoveChassis() needes to be executed every loop
    private void loopMoveChassis(){
        double currentTime = runtime.seconds();
        switch (chassisState) {
            case STOP: {
                setChassisPower(0.0, 0.0, 0.0, 0.0);;
                chassisStartTime = 0.0;
                chassisSetTime = 0.0;
                chassisPower = 0.0;
                telemetry.addData("Move:", "🛑Stop");
                break;
            }
            case FORWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    //setChassisPower(chassisPower, chassisPower, chassisPower, chassisPower);
                    telemetry.addData("Move:","(↑)Forward...");
                }
                break;
            }
            case BACKWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    //setChassisPower(-chassisPower, -chassisPower, -chassisPower, -chassisPower);
                    telemetry.addData("Move:","(↓)Backward...");
                }
                break;
            }
            case LEFTWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    setChassisPower(-chassisPower, chassisPower, chassisPower, -chassisPower);
                    telemetry.addData("Move:","(←)Leftward...");
                }
                break;
            }
            case RIGHTWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    setChassisPower(chassisPower, -chassisPower, -chassisPower, chassisPower);
                    telemetry.addData("Move:","(→)Rightward...");
                }
                break;
            }
            default: {
                setChassisPower(0.0, 0.0, 0.0, 0.0);;
                telemetry.addData("Chassis:", "🛑Stop(Default)");
                break;
            }
        }
    }

    private void forwardState(double seconds, double CPower) {
        chassisState = ChassisState.FORWARD;
        setChassisPower(-chassisPower, -chassisPower, -chassisPower, -chassisPower);
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
    }
    private void backwardState(double seconds, double CPower) {
        chassisState = ChassisState.BACKWARD;
        setChassisPower(chassisPower, chassisPower, chassisPower, chassisPower);
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
    }
    private void leftwardState(double seconds, double CPower) {
        chassisState = ChassisState.LEFTWARD;
        setChassisPower(-chassisPower, chassisPower, chassisPower, -chassisPower);
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
    }
    private void rightwardState(double seconds, double CPower) {
        chassisState = ChassisState.RIGHTWARD;
        setChassisPower(chassisPower, -chassisPower, -chassisPower, chassisPower);
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
    }
    private void stopState() {
        chassisState = ChassisState.STOP;
    }
    private void setChassisPower(double front_left_power, double front_right_power, double rear_left_power, double rear_right_power) {
        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        rear_left.setPower(rear_left_power);
        rear_right.setPower(rear_right_power);
    }
    /*private void stopChassis() {
        setChassisPower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Move:","(S)Stop");
        telemetry.update();
    }*/

    @Override
    public void init() {
        telemetry.addData("初始化" ,"启动");
        init_robot();
        telemetry.addData("初始化", "完毕");
    }

    @Override
    public void start() {
        runtime.reset();// 开始时重置运行时间
    }
    @Override
    public void init_loop() {
        // 初始化循环，这里为空，可能是后续需要添加的功能预留
    }
    @Override
    public void loop() {
        loopMoveChassis();

        int liftCurrentPosition = lift.getCurrentPosition();
        switch (autoState){
            case START0: {
                telemetry.addData("State:", "0 START");
                // Set positions for next state
                // Set forward
                forwardState(0.5, values.AutonomousCPower);
                // Set lift
                top_clip_arm.setPosition(values.armPositions.get("TC_arm_up"));//add
                lift.setTargetPosition(values.liftPositions.get("up"));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                // Change state
                autoState = AutoState.FORWARD_LIFT1;
                break;
            }
            case FORWARD_LIFT1: {
                telemetry.addData("State:", "1 FORWARD & LIFT");
                double lift_current_A = ((DcMotorEx)lift).getCurrent(CurrentUnit.AMPS);
                if(lift_current_A >= 3.0){
                    lift.setPower(0.5);
                }
                //wait for forward & lift
                if (Math.abs(liftCurrentPosition - values.liftPositions.get("up")) < 50 && chassisState==ChassisState.STOP) {
                    // Set positions for next state
                    lift.setTargetPosition(values.liftPositions.get("put"));
                    StartStatePutTime = runtime.seconds();
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                    //change state
                    autoState = AutoState.PUT2;
                }
                break;
            }
            case PUT2: {
                telemetry.addData("State:","2 PUT");

                if(Math.abs(liftCurrentPosition - values.liftPositions.get("put")) < 50&&runtime.seconds()-StartStatePutTime>=5) {
                    top_clip_hand.setPosition(values.clipPositions.get("TC_open"));
                    autoState = AutoState.RETRACT3;
                }
            }
            case RETRACT3:{
                telemetry.addData("State:","3 RETRACT");
                backwardState(0.5,values.AutonomousCPower);
                lift.setTargetPosition(values.liftPositions.get("zero"));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                top_clip_arm.setPosition(values.armPositions.get("TC_arm_down"));
                autoState = AutoState.WAIT4;
            }
            case WAIT4: {
                telemetry.addData("State:", "4 WAIT");
                if(chassisState==ChassisState.STOP) {
                    rightwardState(0.5,values.AutonomousCPower);
                    autoState = AutoState.END5;
                }

                //stopState();
                //break;
            }
            case END5:{
                telemetry.addData("State:","5 END");
                stopState();
                break;
            }
            default:{
                telemetry.addData("State:", "WAIT(default)");
                stopState();
            }
        }

        telemetry.addData("运行时间", runtime);
        telemetry.addData("角度(Degrees)", "(%.2f)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("抬升位置", "(%d)", liftCurrentPosition);
    }
    
}
