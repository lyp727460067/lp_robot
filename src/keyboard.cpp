/*
 * Copyright (c) 2013, HUST-Renesas Lab, Huazhong University Of Science And
 * Technology Copyright (c) 2010, Arizona Robotics Research Group, University of
 * Arizona Copyright (c) 2008, Willow Garage, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * =====================================================================================
 *        COPYRIGHT NOTICE
 *        Copyright (c) 2013  HUST-Renesas Lab
 *        ALL rights reserved.
 *//**        
 *        @file     keyboard.cpp
 *        @brief    robot keyboard control
 *        @version  0.1
 *        @date     2013/5/23 15:22:40
 *        @author   Hu Chunxu , huchunxu@hust.edu.cn
 *//* ==================================================================================
 *  @0.1    Hu Chunxu    2013/5/23 15:22:40   create orignal file
 * =====================================================================================
 */

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <signal.h>
#include <signal.h>  // signal functions
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <termios.h>
#include "sensor_msgs/Joy.h"
#include <boost/thread/thread.hpp>
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

/* 带有shift键 */
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class SmartCarKeyboardTeleopNode {
 private:
  double walk_vel_;
  double run_vel_;
  double yaw_rate_;
  double yaw_rate_run_;
  sensor_msgs::Joy joy_;
  geometry_msgs::Twist cmdvel_;

  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_joy_;

 public:
  boost::thread t;
  SmartCarKeyboardTeleopNode() {
    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_joy_ = n_.advertise<sensor_msgs::Joy>("/joy", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel_, 0.5);
    n_private.param("run_vel", run_vel_, 1.0);
    n_private.param("yaw_rate", yaw_rate_, 1.0);
    n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
  }

  ~SmartCarKeyboardTeleopNode() {}
  void keyboardLoop();

  void stopRobot() {
    cmdvel_.linear.x = 0.0;
    cmdvel_.angular.z = 0.0;
    pub_.publish(cmdvel_);
  }
};

SmartCarKeyboardTeleopNode* tbk1;

/**
 * 文件描述符
 * 内核（kernel）利用文件描述符（file
 * descriptor）来访问文件。文件描述符是非负整数。 标准输入（standard
 * input）的文件描述符是 0，标准输出（standard output）是 1，标准错误（standard
 * error）是 2。
 */
int kfd = 0;

/**
 *  === struct termios ===
 *  tcflag_t c_iflag;  输入模式
 *　tcflag_t c_oflag;  输出模式
 *　tcflag_t c_cflag;  控制模式
 *  tcflag_t c_lflag;  本地模式
 *  cc_t c_cc[NCCS];   控制字符
 */
struct termios cooked, raw;
bool done;
static void termin_out(int sig) {
  if (sig == SIGINT) {
    ros::shutdown();
    // tbk1->t.interrupt();
    // tbk1->t.join();
    // tbk1->stopRobot();

    /* 设置终端参数 */
    // tcsetattr(kfd, TCSANOW, &cooked);
    // puts("terminal resume");
  }
  // exit(1);
  // return;
}

int main(int argc, char** argv) {
  ros::init(
      argc, argv, "tbk",
      ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  SmartCarKeyboardTeleopNode tbk;
  tbk1 = &tbk;
  signal(SIGINT, termin_out);
  signal(SIGTERM, termin_out);
  /* 创建一个新的线程 */
  tbk.t = boost::thread(
      boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
  // while(ros::ok()){
  //	ros::spinOnce();
  // }
  ros::spin();

  tbk.t.interrupt();
  tbk.t.join();
  tbk.stopRobot();

  /* 设置终端参数 */
  tcsetattr(kfd, TCSANOW, &cooked);
  puts("terminal resume");
  return (0);
}

int pub_joy = 0;
void SmartCarKeyboardTeleopNode::keyboardLoop() {
  char c;
  double max_tv = walk_vel_;
  double max_rv = yaw_rate_;
  bool dirty = false;
  bool pub = false;
  int speed = 0;
  int turn = 0;
  joy_.buttons.resize(8);
  std::cout<<  joy_.buttons.size()<<std::endl;
  /**
   * 从终端中获取按键
   * int tcgetattr(int fd, struct termios *termios_p);
   */
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));

  /**
   * c_lflag : 本地模式标志，控制终端编辑功能
   * ICANON: 使用标准输入模式
   * ECHO: 显示输入字符
   */
  raw.c_lflag &= ~(ICANON | ECHO);

  /**
   * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
   * VEOL: 附加的End-of-file字符
   * VEOF: End-of-file字符
   * */
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("Use WASD keys to control the robot");
  puts("Press Shift to move faster");

  /* *
   * struct pollfd {
　　       int fd;        文件描述符
　       　short events;  等待的事件
　　       short revents; 实际发生了的事件
　       　};
*/
  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;

  for (;;) {
    boost::this_thread::interruption_point();

    /* get the next event from the keyboard */
    int num;

    /**
     * poll:把当前的文件指针挂到设备内部定义的等待队列中。
     * unsigned int (*poll)(struct file * fp, struct poll_table_struct * table)
     */
    if ((num = poll(&ufd, 1, 250)) < 0) {
      /**
       * perror( ) 用来将上一个函数发生错误的原因输出到标准设备(stderr)。
       * 参数s所指的字符串会先打印出,后面再加上错误原因字符串。
       * 此错误原因依照全局变量errno 的值来决定要输出的字符串。
       * */
      perror("poll():");
      return;
    } else if (num > 0) {
      if (read(kfd, &c, 1) < 0) {
        perror("read():");
        return;
      }
    } else {
      /* 每按下一次动一下 */
      if (dirty == true) {
        stopRobot();
        dirty = false;
      }

      continue;
    }

    switch (c) {
      case 'w':
        max_tv = walk_vel_;
        speed = 1;
        turn = 0;
        pub = true;
        // dirty = true;
        break;
      case 's':
        max_tv = walk_vel_;
        speed = -1;
        turn = 0;
        pub = true;
        // dirty = true;
        break;
      case 'a':
        max_rv = yaw_rate_;
        speed = 0;
        turn = 1;
        pub = true;
        dirty = true;
        break;
      case 'd':
        max_rv = yaw_rate_;
        speed = 0;
        turn = -1;
        pub = true;
        dirty = true;
        break;

      case 'W':
        max_tv = run_vel_;
        speed = 1;
        turn = 0;
        pub = true;
        // dirty = true;
        break;
      case 'S':
        max_tv = run_vel_;
        speed = -1;
        turn = 0;
        pub = true;
        // dirty = true;
        break;
      case 'A':
        max_rv = yaw_rate_run_;
        speed = 0;
        turn = 1;
        pub = true;
        dirty = true;
        break;
      case 'D':
        max_rv = yaw_rate_run_;
        speed = 0;
        turn = -1;
        pub = true;
        dirty = true;
        break;
      case 'q':
      case 'Q':
      case 'E':
      case 'e':
      case 'F':
      case 'f':
        speed = 0;
        turn = 0;
        dirty = true;
        pub = true;
        break;

      default:
        break;
        // max_tv = walk_vel_;
        // max_rv = yaw_rate_;
        // speed = 0;
        // turn = 0;
        // dirty = false;
    }

    switch (c) {
      case 'b':
      case 'B':
        pub_joy = 1;
        joy_.buttons[1] = 1;
        break;
      case 'y':
      case 'Y':

        pub_joy = 1;
        joy_.buttons[3] = 1;
        break;
      case 'x':
      case 'X':
        joy_.buttons[2] = 1;
        pub_joy = 1;
        break;
      case 't':
      case 'T':
        joy_.buttons[7] = 1;
        pub_joy = 1;
        break;
      default:
        break;
    }

    if (pub) {
      pub = false;
      cmdvel_.linear.x = speed * max_tv;
      cmdvel_.angular.z = turn * max_rv;
      pub_.publish(cmdvel_);
    }
    if (pub_joy) {
      pub_joy = 0;
      // if (pub_joy == 2) {

      // }
      // if (pub_joy == 1) {
      //   pub_joy = 2;
      // }
      joy_.header.frame_id = "base";
      joy_.header.stamp = ros::Time::now();
      pub_joy_.publish(joy_);
      usleep(100000);
      joy_.buttons[1] = 0;
      joy_.buttons[3] = 0;
      joy_.buttons[2] = 0;
      joy_.buttons[7] = 0;
      pub_joy_.publish(joy_);
    }
  }
}
