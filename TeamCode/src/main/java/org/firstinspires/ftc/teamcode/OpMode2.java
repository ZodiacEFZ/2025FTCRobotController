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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "遥控测试v3.4.1(Down Clip)", group = "Iterative OpMode")
public class OpMode2 extends OpMode {
    // 记录运行时间的计时器
    private final ElapsedTime runtime = new ElapsedTime();
    //底部夹子的两个舵机
    //down_clip_head控制夹子的旋转
    //down_clip_hand控制夹子的抓放
    private Servo down_clip_head;
    private Servo down_clip_hand;
    private ServoImplEx down_clip_arm;
    //舵机夹子,顶部的那个,目前还没装好
    private Servo clip;
    private Servo top_clip_hand;
    private Servo top_clip_head;
    // 四个底盘电机
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;

    //抬升电机
    private DcMotor lift;
    // 惯性测量单元
    private IMU imu;
    private double multiplier;

    private Values values = new Values();

    private boolean DCstate = false, LB_last_pressed = false; // false for open; true for close

    @Override
    public void init() {
        multiplier=1.1;
        // 向遥测发送初始化开始信息
        telemetry.addData("初始化" ,"启动");
        //初始化电机
        {
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            down_clip_arm=(ServoImplEx) hardwareMap.get(Servo.class,"DownClipArm");
            down_clip_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
            top_clip_hand=hardwareMap.get(Servo.class,"TopClipHand");
            top_clip_head=hardwareMap.get(Servo.class,"TopClipHead");
//            down_clip_head.setPosition(0);
//            down_clip_hand.setDirection(Servo.Direction.REVERSE);
            //舵机夹子
            clip = hardwareMap.get(Servo.class,"Clip");
            clip.setPosition(0);
//            top_clip_hand.setPosition(0);

            //Servo.Direction.REVERSE
            //抬升电机
            lift = hardwareMap.get(DcMotor.class,"Lift");
            // 从硬件映射中获取四个底盘电机
            front_left = hardwareMap.get(DcMotor.class, "frontLeft");
            front_right = hardwareMap.get(DcMotor.class, "frontRight");
            rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
            rear_right = hardwareMap.get(DcMotor.class, "rearRight");
            // 设置电机的转动方向
            lift.setDirection(DcMotor.Direction.REVERSE);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.REVERSE);
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            rear_right.setDirection(DcMotor.Direction.REVERSE);
            // 设置电机在功率为零时的行为为制动
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }
    private void FieldCentricMecanum(){
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // 如果按下游戏手柄的选项按钮，则重置 IMU 的偏航角
        if(gamepad1.options){
            telemetry.addData("按键","[START]");
            imu.resetYaw();
            gamepad1.rumble(100);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // 将运动方向相对于机器人的旋转进行旋转
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        // 抵消不完美的平移
        rotX = rotX * multiplier;
        //分母是最大的电机功率（绝对值）或1，这确保所有功率保持相同的比例，但仅当至少一个超出范围[-1,1]时。
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        // 计算四个电机的功率
        double front_left_power = (rotY + rotX + rx) / denominator;
        double rear_left_power = (rotY - rotX + rx) / denominator;
        double front_right_power = (rotY - rotX - rx) / denominator;
        double rear_right_power = (rotY + rotX - rx) / denominator;
        // 设置四个电机的功率
        front_left.setPower(front_left_power);
        rear_left.setPower(rear_left_power);
        front_right.setPower(front_right_power);
        rear_right.setPower(rear_right_power);

        telemetry.addData("底盘方向", "(%.2f)rad", botHeading);
        telemetry.addData("底盘功率", "左前:(%.2f) 右前:(%.2f) 左后:(%.2f) 右后:(%.2f)", front_left_power, front_right_power, rear_left_power, rear_right_power);
    }

    private void LiftLoop(){
        if(gamepad2.y){     //Lift up
            lift.setTargetPosition(values.liftPositions.get("up"));
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            telemetry.addData("按键2","[Y]");
        }
        if(gamepad2.a){     //Lift zero
            lift.setTargetPosition(values.liftPositions.get("zero"));
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            telemetry.addData("按键2","[A]");
        }
        if(gamepad2.b){     //Force stop Lift
            lift.setPower(0);
            telemetry.addData("按键2","[B]");
        }
        //lift.setPower((rt>0.1?rt-0.1:0)-(lt>0.1?lt-0.1:0));

        telemetry.addData("抬升编码器",lift.getCurrentPosition());
        telemetry.addData("抬升功率",lift.getPower());
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
        }
        else if(!gamepad2.left_bumper){
            LB_last_pressed = false;
        }

        //DownClip head control
        double lx2 = gamepad2.left_stick_x;
        double DC_head_curr = down_clip_head.getPosition();
        if(Math.abs(lx2) > 0.1){
            double DC_head_set = DC_head_curr + lx2/50;
            DC_head_set = Math.min(DC_head_set, 1);
            DC_head_set = Math.max(DC_head_set, 0);
            down_clip_head.setPosition(DC_head_set);
        }

        //DownClip arm control
        double ly2 = gamepad2.left_stick_y;
        double DC_arm_curr = down_clip_arm.getPosition();
        if(Math.abs(ly2) > 0.1){
            double DC_arm_set = DC_arm_curr + ly2/50;
            DC_arm_set = Math.min(DC_arm_set, 1);
            DC_arm_set = Math.max(DC_arm_set, 0);
            down_clip_arm.setPosition(DC_arm_set);
        }

        //TopClip control 等待完成
        if(gamepad2.right_stick_x>0.1) {
            top_clip_head.setPosition(Math.min(1,Math.max(0,top_clip_head.getPosition() + gamepad2.right_stick_x / 100)));
        }
        if(gamepad2.right_stick_x<-0.1) {
            top_clip_head.setPosition(Math.min(1,Math.max(0,top_clip_head.getPosition()+gamepad2.right_stick_x / 100)));
        }
        if(gamepad2.right_stick_y>0.1) {
            top_clip_hand.setPosition(Math.min(0.75, Math.max(0.45, top_clip_hand.getPosition() + gamepad2.right_stick_y / 200)));
        }
        if(gamepad2.right_stick_y<-0.1) {
            top_clip_hand.setPosition(Math.min(0.75,Math.max(0.45,top_clip_hand.getPosition() + gamepad2.right_stick_y / 200)));
        }

        if(gamepad2.left_bumper) {
//            clip.setPosition(0);//0degrees
            clip.setPosition(down_clip_hand.getPosition()+0.001);
        }
        if(gamepad2.right_bumper) {
//            clip.setPosition(0.5);//90degrees
            clip.setPosition(down_clip_hand.getPosition()-0.001);
        }


        telemetry.addData("DownClip","[Head:%.2f] [Arm:%.2f] [Hand:%.2f]", down_clip_head.getPosition(),down_clip_arm.getPosition(),down_clip_hand.getPosition());
        telemetry.addData("TopClip","[Clip:%.2f] [Head:%.2f] [Hand:%.2f]",clip.getPosition(),top_clip_head.getPosition(),top_clip_hand.getPosition());
    }

    @Override
    public void loop() {
        FieldCentricMecanum();

        ClipsLoop();

        LiftLoop();

        telemetry.addData("运行时间", runtime);
    }
    @Override
    public void stop() {
        // 停止时的操作，这里为空，可能是后续需要添加的功能预留
    }
}