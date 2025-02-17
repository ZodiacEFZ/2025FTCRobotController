/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "01用这个 遥控v6.0.0 (FieldCentric)", group = "TeleOp")
public class OpMode2 extends OpMode {
    // 记录运行时间的计时器
    private final ElapsedTime runtime = new ElapsedTime();
    //底部夹子的两个舵机
    //down_clip_head控制夹子的旋转
    //down_clip_hand控制夹子的抓放
    private Servo down_clip_head;
    private Servo down_clip_hand;
    private ServoImplEx down_clip_arm;

    private Servo top_clip_hand;
    private Servo top_clip_arm;
    //private Servo top_clip_head;

    private Servo head;
    // 四个底盘电机
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private DcMotor intake;
    //抬升电机
    private DcMotor lift;
    // 惯性测量单元
    private IMU imu;
    //private double multiplier;

    private Values values = new Values();

    private boolean DCstate = true, LB_last_pressed = false; // false for open; true for close
    private boolean TCstate = true, RB_last_pressed = false; // false for open; true for close
    private double RB_press_start_time = 0.0;

    private boolean HeadState = true, gp1_LB_last_pressed = false; // true for up; false for down

    private enum LiftState {
        ZERO,
        UP,
        PUT,
        MAX,
    };
    private LiftState liftState = LiftState.ZERO;
    private boolean gp2y_last_pressed = false;

    /*
    private enum IntakeState {
        IN,
        OUT,
    };
    private IntakeState intakeState = IntakeState.IN;*/
    double intake_big_current_time = 0.0;

    @Override
    public void init() {
        //multiplier=1.1;
        // 向遥测发送初始化开始信息
        telemetry.addData("初始化" ,"启动");
        //初始化电机
        {
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            down_clip_arm=(ServoImplEx) hardwareMap.get(Servo.class,"DownClipArm");
            down_clip_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));

            top_clip_arm=hardwareMap.get(Servo.class,"TopClipArm");
            //top_clip_head=hardwareMap.get(Servo.class,"TopClipHead");
            top_clip_hand = hardwareMap.get(Servo.class,"TopClipHand");

            head = hardwareMap.get(Servo.class,"Head");

            intake = hardwareMap.get(DcMotor.class,"Intake");
            //抬升电机
            lift = hardwareMap.get(DcMotor.class,"Lift");
            // 从硬件映射中获取四个底盘电机
            front_left = hardwareMap.get(DcMotor.class, "frontLeft");
            front_right = hardwareMap.get(DcMotor.class, "frontRight");
            rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
            rear_right = hardwareMap.get(DcMotor.class, "rearRight");
            // 设置电机的转动方向
            intake.setDirection(DcMotor.Direction.FORWARD);
            lift.setDirection(DcMotor.Direction.REVERSE);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.REVERSE);
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            rear_right.setDirection(DcMotor.Direction.REVERSE);
            // 设置电机在功率为零时的行为为制动
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setTargetPosition(0);
        }
        {
            // 从硬件映射中获取 IMU
            imu = hardwareMap.get(IMU.class, "imu");
            // 调整方向参数以匹配机器人
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // 如果没有这个，REV Hub 的方向将被假定为标志朝上 / USB 朝前
            imu.initialize(parameters);
            // 重置 IMU 的偏航角
            imu.resetYaw();
        }

        // 向遥测发送初始化完成信息
        telemetry.addData("初始化", "完毕");
    }
    @Override
    public void init_loop() {
        // 初始化循环，这里为空，可能是后续需要添加的功能预留
    }
    @Override
    public void start() {
        // 开始时重置运行时间
        runtime.reset();

        down_clip_hand.setPosition(values.clipPositions.get("DC_close"));
        top_clip_hand.setPosition(values.clipPositions.get("TC_close"));
        down_clip_arm.setPosition(values.armPositions.get("DC_arm_init"));
        down_clip_head.setPosition(values.armPositions.get("DC_head_init"));
        head.setPosition(values.headPositions.get("Head_down"));
        HeadState = false;
    }

    private void HeadLoop(){
        if(gamepad1.left_bumper && !gp1_LB_last_pressed){
            HeadState = !HeadState;
            gp1_LB_last_pressed = true;
            if(HeadState){
                head.setPosition(values.headPositions.get("Head_up"));
            }
            else {
                head.setPosition(values.headPositions.get("Head_down"));
            }
        }
        else if(!gamepad1.left_bumper){
            gp1_LB_last_pressed = false;
        }
    }

    private void FieldCentricMecanum(){
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (gamepad2.dpad_left){
            rx += 0.5;
        }
        if (gamepad2.dpad_right){
            rx -= 0.5;
        }
        rx = Math.min(rx, 1);
        rx = Math.max(rx, -1);

        // 如果按下游戏手柄的选项按钮，则重置 IMU 的偏航角
        if(gamepad1.options){
            telemetry.addData("按键","[START]");
            imu.resetYaw();
            gamepad1.rumble(1000);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // 将运动方向相对于机器人的旋转进行旋转
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        // 抵消不完美的平移
        rotX = rotX * values.chassisMultiplier;
        //分母是最大的电机功率（绝对值）或1，这确保所有功率保持相同的比例，但仅当至少一个超出范围[-1,1]时。
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        // 计算四个电机的功率
        double front_left_power = (rotY + rotX + rx) / denominator;
        double rear_left_power = (rotY - rotX + rx) / denominator;
        double front_right_power = (rotY - rotX - rx) / denominator;
        double rear_right_power = (rotY + rotX - rx) / denominator;

        if(gamepad2.x){
            front_left_power = rear_left_power = front_right_power = rear_right_power = 0.0;
        }

        // 设置四个电机的功率
        front_left.setPower(front_left_power);
        rear_left.setPower(rear_left_power);
        front_right.setPower(front_right_power);
        rear_right.setPower(rear_right_power);

        telemetry.addData("底盘方向", "(%.2f)rad", botHeading);
        telemetry.addData("底盘功率", "左前:(%.2f) 右前:(%.2f) 左后:(%.2f) 右后:(%.2f)", front_left_power, front_right_power, rear_left_power, rear_right_power);
    }

    private void IntakeLoop(){
        boolean pad1y = (gamepad1.y || gamepad2.dpad_up);
        boolean pad1a = (gamepad1.a || gamepad2.dpad_down);
        int intake_cur_pos=intake.getCurrentPosition();
        telemetry.addData("IntakeEncoder",intake_cur_pos);
        int intake_set_pos=((pad1y?1:0)-(pad1a?1:0))*values.intSpeed.get("intake")+intake_cur_pos;
        if(intake_set_pos!=intake_cur_pos) {
            intake.setTargetPosition(intake_set_pos);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(0.1);

            /*if(intake_set_pos > intake_cur_pos){
                intakeState = IntakeState.OUT;
            }
            else{
                intakeState = IntakeState.IN;
            }*/
        }

        if(gamepad1.b){     //Force stop Intake
            intake.setPower(0);
            telemetry.addData("按键","[B]");
        }

        double intake_current_A = ((DcMotorEx)intake).getCurrent(CurrentUnit.AMPS);
        if(intake_current_A >= 2.0){
            if((runtime.seconds() - intake_big_current_time) > 2.0){
                intake.setPower(0.0);
                intake.setTargetPosition(intake_cur_pos);
            }
            else if(intake_big_current_time <= 0.001){
                intake_big_current_time = runtime.seconds();
            }
        }
        if(intake_current_A < 0.5){
            intake_big_current_time = 0.0;
            intake.setPower(0.1);
        }

        telemetry.addData("Intake编码器",intake_cur_pos);
        telemetry.addData("Intake目标",intake.getTargetPosition());
        telemetry.addData("Intake速度",((DcMotorEx)intake).getVelocity());
        telemetry.addData("Intake电流",((DcMotorEx) intake).getCurrent(CurrentUnit.AMPS));
    }

    private void LiftLoop(){
        double lt2 = gamepad2.left_trigger;
        double rt2 = gamepad2.right_trigger;
        int lift_cur_pos = lift.getCurrentPosition();
        if(Math.abs(lt2) > 0.1){
            liftState = LiftState.ZERO;

            int lift_set_pos = lift_cur_pos + (int) (lt2*50);
            lift_set_pos = Math.min(lift_set_pos, values.liftPositions.get("max"));
            //lift_set_pos = Math.max(lift_set_pos, values.liftPositions.get("zero"));
            lift.setTargetPosition(lift_set_pos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }
        if(Math.abs(rt2) > 0.1){
            liftState = LiftState.ZERO;

            int lift_set_pos = lift_cur_pos - (int) (rt2*50);
            lift_set_pos = Math.min(lift_set_pos, values.liftPositions.get("max"));
            //lift_set_pos = Math.max(lift_set_pos, values.liftPositions.get("zero"));
            lift.setTargetPosition(lift_set_pos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }

        if(gamepad2.y && !gp2y_last_pressed){     //Lift up
            gp2y_last_pressed = true;
            telemetry.addData("按键2","[Y]");
            if((liftState == LiftState.ZERO) || (liftState == LiftState.PUT)){
                lift.setTargetPosition(values.liftPositions.get("up"));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);

                top_clip_arm.setPosition(values.armPositions.get("TC_arm_up"));

                liftState = LiftState.UP;
            }
            else if (liftState == LiftState.UP){
                liftState = LiftState.PUT;

                lift.setTargetPosition(values.liftPositions.get("put"));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);

                top_clip_arm.setPosition(values.armPositions.get("TC_arm_up"));

            }
        }
        else if (!gamepad2.y){
            gp2y_last_pressed = false;
        }

        if(gamepad2.a){     //Lift zero
            liftState = LiftState.ZERO;

            lift.setTargetPosition(values.liftPositions.get("zero"));
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);

            top_clip_arm.setPosition(values.armPositions.get("TC_arm_down"));

            telemetry.addData("按键2","[A]");
        }

        if(gamepad2.b){     //Force stop Lift & reset
            lift.setPower(0);
            //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("按键2","[B]");
            gamepad2.rumble(1000);
        }
        //lift.setPower((rt>0.1?rt-0.1:0)-(lt>0.1?lt-0.1:0));

        double lift_current_A = ((DcMotorEx)lift).getCurrent(CurrentUnit.AMPS);
        if(lift_current_A >= 3.0){
            lift.setPower(0.5);
        }
        if(lift_current_A >= 5.0){
            lift.setPower(0.0);
        }

        telemetry.addData("抬升编码器",lift_cur_pos);
        telemetry.addData("抬升目标",lift.getTargetPosition());
        telemetry.addData("抬升速度",((DcMotorEx)lift).getVelocity());
        telemetry.addData("抬升电流",lift_current_A);
    }

    private void ClipsLoop(){
        //DownClip hand control
        if(gamepad2.left_bumper && !LB_last_pressed){
            DCstate = !DCstate;
            LB_last_pressed = true;
            if(DCstate){
                down_clip_hand.setPosition(values.clipPositions.get("DC_close"));
            }
            else {
                down_clip_hand.setPosition(values.clipPositions.get("DC_open"));
            }
            gamepad1.rumble(100);
        }
        else if(!gamepad2.left_bumper){
            LB_last_pressed = false;
        }

        //DownClip head control
        double lx2 = gamepad2.left_stick_x;
        double DC_head_curr = down_clip_head.getPosition();
        if(Math.abs(lx2) > 0.1){
            double DC_head_set = DC_head_curr + lx2/30;
            DC_head_set = Math.min(DC_head_set, 1);
            DC_head_set = Math.max(DC_head_set, 0);
            down_clip_head.setPosition(DC_head_set);
        }

        //DownClip arm control
        double ly2 = -gamepad2.left_stick_y;
        double DC_arm_curr = down_clip_arm.getPosition();
        if(Math.abs(ly2) >= 0.2){
            double DC_arm_set = DC_arm_curr + ly2*values.speed.get("down_clip_arm_ly2");
            DC_arm_set = Math.min(DC_arm_set, 1);
            DC_arm_set = Math.max(DC_arm_set, 0);
            down_clip_arm.setPosition(DC_arm_set);
        }
        /*if(ly2 > 0.8){
            //down_clip_arm.setPosition(values.armPositions.get("DC_arm_up"));
            down_clip_arm.setPosition(values.armPositions.get("DC_arm_init"));
        }
        if(ly2 < -0.8){
            down_clip_arm.setPosition(values.armPositions.get("DC_arm_down"));
        }*/
        /*if((intakeState == IntakeState.IN) && (DC_arm_curr < values.armPositions.get("DC_arm_IN_min"))){
            down_clip_arm.setPosition(values.armPositions.get("DC_arm_IN_min"));
        }*/

        //DownClip arm 放下
        /*if(gamepad2.dpad_down){
            telemetry.addData("按键2","[↓]");
            down_clip_arm.setPosition(values.armPositions.get("DC_arm_down"));
        }*/

        //DownClip 和 TopClip 交接 不做

        //TopClip hand control
        if(gamepad2.right_bumper && !RB_last_pressed){
            RB_press_start_time = runtime.seconds();
            TCstate = !TCstate;
            RB_last_pressed = true;
            if(TCstate){
                top_clip_hand.setPosition(values.clipPositions.get("TC_close"));
            }
            else {
                if(liftState != LiftState.UP){
                    top_clip_hand.setPosition(values.clipPositions.get("TC_open"));
                }
            }
            gamepad1.rumble(100);
        }
        else if(!gamepad2.right_bumper){
            RB_last_pressed = false;
            RB_press_start_time = 0.0;
        }
        else if((RB_press_start_time > 0.01) && ((runtime.seconds()-RB_press_start_time)>=2.0)){
            top_clip_hand.setPosition(values.clipPositions.get("TC_open"));
        }

        //TopClip arm
        double ry2 = gamepad2.right_stick_y;
        double TC_arm_curr = top_clip_arm.getPosition();
        if(Math.abs(ry2) > 0.1){
            double TC_arm_set = TC_arm_curr - ry2/100;
            TC_arm_set = Math.min(TC_arm_set, 1);
            TC_arm_set = Math.max(TC_arm_set, 0);
            top_clip_arm.setPosition(TC_arm_set);
        }

        //TopClip head
        /*double rx2 = gamepad2.right_stick_x;
        double TC_head_curr = top_clip_head.getPosition();
        if(Math.abs(rx2) > 0.1){
            double TC_head_set = TC_head_curr + rx2/100;
            TC_head_set = Math.min(TC_head_set, 1);
            TC_head_set = Math.max(TC_head_set, 0);
            top_clip_head.setPosition(TC_head_set);
        }*/

        telemetry.addData("DownClip","[Head:%.2f] [Arm:%.2f] [Hand:%.2f]", down_clip_head.getPosition(),down_clip_arm.getPosition(),down_clip_hand.getPosition());
        telemetry.addData("DownClip status:",DCstate);
        //telemetry.addData("TopClip","[Hand:%.2f] [Head:%.2f] [Arm:%.2f]",top_clip_hand.getPosition(),top_clip_head.getPosition(),top_clip_arm.getPosition());
        telemetry.addData("TopClip","[Hand:%.2f] [Arm:%.2f]",top_clip_hand.getPosition(),top_clip_arm.getPosition());
        telemetry.addData("TopClip status:",TCstate);
    }

    @Override
    public void loop() {
        HeadLoop();

        FieldCentricMecanum();

        ClipsLoop();

        IntakeLoop();

        LiftLoop();

        telemetry.addData("运行时间", runtime);
    }
    @Override
    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
        lift.setPower(0);
        intake.setPower(0);
    }
}