"""
ROS2 Action 服务器 - 机械臂控制服务器
功能：接收客户端的运动指令，验证并执行，在执行过程中发送反馈，完成后返回结果
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse  # Action服务器和响应类型
from rclpy.callback_groups import ReentrantCallbackGroup  # 可重入回调组（支持并发）
from rclpy.executors import MultiThreadedExecutor  # 多线程执行器
from wlkata_interfaces.action import MoveArm  # 机械臂运动Action接口
from wlkata_interfaces.msg import ToolConfig  # 工具配置消息类型
import wlkatapython
import serial
import sys
import time

class ArmActionServer:
    """
    机械臂Action服务器类
    负责处理来自客户端的运动指令请求
    """
    
    def __init__(self, node):
        """
        初始化Action服务器
        
        参数:
            node: ROS2节点对象
        """
        self.node = node
        
        # 创建Action服务器
        # 
        # ========== 回调函数执行顺序和关系 ==========
        # 
        # ROS2 Action服务器的生命周期和回调执行流程：
        # 
        # 1. 【goal_callback】- 目标验证阶段（最先执行）
        #    - 触发时机：客户端发送目标时立即调用
        #    - 作用：验证目标是否有效（参数检查、合法性验证等）
        #    - 返回值：
        #      * GoalResponse.ACCEPT → 接受目标，进入下一步
        #      * GoalResponse.REJECT → 拒绝目标，流程终止，不会调用execute_callback
        #    - 特点：同步执行，必须快速返回，不应执行耗时操作
        # 
        # 2. 【handle_accepted_callback】（可选，当前代码中未使用）
        #    - 触发时机：goal_callback返回ACCEPT后立即调用
        #    - 作用：目标被接受后的预处理（如资源分配、状态初始化等）
        #    - 注意：如果未提供此回调，目标被接受后直接进入execute_callback
        # 
        # 3. 【execute_callback】- 任务执行阶段（目标被接受后执行）
        #    - 触发时机：目标被接受后，在新线程中异步执行
        #    - 作用：实际执行任务（这里是控制机械臂运动）
        #    - 执行特点：
        #      * 在独立线程中运行，不会阻塞其他回调
        #      * 可以长时间运行（执行任务）
        #      * 在执行过程中可以：
        #        - 调用 goal_handle.publish_feedback() 发送进度反馈
        #        - 检查 goal_handle.is_cancel_requested 判断是否被取消
        #        - 调用 goal_handle.succeed()/canceled()/abort() 结束任务
        # 
        # 4. 【cancel_callback】- 取消请求处理（可能在执行过程中调用）
        #    - 触发时机：客户端请求取消任务时调用（可能在execute_callback执行期间）
        #    - 作用：决定是否接受取消请求
        #    - 返回值：
        #      * CancelResponse.ACCEPT → 接受取消，execute_callback中的is_cancel_requested变为True
        #      * CancelResponse.REJECT → 拒绝取消，任务继续执行
        #    - 注意：
        #      * 此回调与execute_callback可能并发执行（不同线程）
        #      * execute_callback需要主动检查is_cancel_requested来处理取消
        # 
        # ========== 典型执行流程示例 ==========
        # 
        # 正常执行流程：
        #   客户端发送目标 
        #   → goal_callback (验证) 
        #   → 返回ACCEPT 
        #   → execute_callback (执行任务)
        #   → 执行过程中定期发送反馈
        #   → 执行完成，调用goal_handle.succeed()
        # 
        # 被拒绝的流程：
        #   客户端发送目标 
        #   → goal_callback (验证) 
        #   → 返回REJECT 
        #   → 流程终止（不执行execute_callback）
        # 
        # 被取消的流程：
        #   客户端发送目标 
        #   → goal_callback (验证) → ACCEPT
        #   → execute_callback (开始执行)
        #   → 客户端发送取消请求
        #   → cancel_callback (处理取消) → ACCEPT
        #   → execute_callback中检测到is_cancel_requested=True
        #   → execute_callback调用goal_handle.canceled()并返回
        # 
        # ========== 参数说明 ==========
        #   node: ROS2节点
        #   MoveArm: Action类型
        #   'move_arm': Action名称（客户端必须使用相同名称）
        #   goal_callback: 目标验证回调（决定是否接受目标）
        #   execute_callback: 执行回调（实际执行任务）
        #   cancel_callback: 取消回调（处理取消请求）
        #   callback_group: 回调组（ReentrantCallbackGroup允许并发处理多个目标）
        self._action_server = ActionServer(
            node,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,  # 目标验证回调
            # handle_accepted_callback: 可选，目标被接受后的回调
            execute_callback=self.execute_callback,  # 任务执行回调
            cancel_callback=self.cancel_callback,  # 取消请求回调
            callback_group=ReentrantCallbackGroup()  # 可重入回调组，支持并发
        )
        self.wlkata_arm=wlkatapython.Harobot_UART()
        self.sre = None  # 初始化为None，便于后续检查
        try:
            self.sre=serial.Serial("/dev/ttyUSB0",115200)
            self.wlkata_arm.init(self.sre,-1)
            self.node.get_logger().info('串口打开成功')
        except Exception as e:
            self.node.get_logger().error(f"串口打开失败: {e}")
            # 如果串口打开失败，清理已创建的资源
            if self.sre is not None:
                try:
                    self.sre.close()
                except:
                    pass
            sys.exit(1)

        self.node.get_logger().info('服务已启动')

    def goal_callback(self, goal_request):
        """
        目标验证回调函数
        
        【执行顺序】：第1步 - 最先执行
        【触发时机】：客户端发送目标时立即同步调用
        【执行线程】：主线程（同步执行，必须快速返回）
        【后续流程】：
            - 返回ACCEPT → 会调用execute_callback（在新线程中异步执行）
            - 返回REJECT → 流程终止，不会调用execute_callback
        
        参数:
            goal_request: 客户端发送的目标请求对象
            
        返回:
            GoalResponse.ACCEPT: 接受目标，后续会调用execute_callback执行任务
            GoalResponse.REJECT: 拒绝目标，流程终止，不执行任务
        """

        self.node.get_logger().info(f'开始校验消息')
        
        # ========== 验证运动模式 ==========
        # 模式值：0=点到点运动，1=直线运动，2=圆弧运动
        # 只接受有效的模式值
        if goal_request.mode not in [0, 1, 2]:
            self.node.get_logger().error(f'运动模式： {goal_request.mode}')
            return GoalResponse.REJECT  # 拒绝无效模式
        
        # ========== 验证速度范围 ==========
        # 速度必须在0.0到1.0之间
        if not (0.0 <= goal_request.speed <= 1.0):
            self.node.get_logger().error(f'速度值： {goal_request.speed}')
            return GoalResponse.REJECT  # 拒绝超出范围的速度

        if goal_request.absolute not in [0, 1]:
            self.node.get_logger().error(f'运动模式2： {goal_request.absolute}')
            return GoalResponse.REJECT  # 判断绝对还是增量
        
        # 所有验证通过，接受目标
        self.node.get_logger().info(f'验证通过')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        取消请求回调函数
        
        【执行顺序】：可能在execute_callback执行期间调用
        【触发时机】：客户端请求取消任务时立即调用
        【执行线程】：独立线程（可能与execute_callback并发执行）
        【与execute_callback的关系】：
            - 如果返回ACCEPT，execute_callback中的goal_handle.is_cancel_requested会变为True
            - execute_callback需要主动检查is_cancel_requested来处理取消
            - 这两个回调可能在不同线程中同时运行，需要注意线程安全
        
        参数:
            goal_handle: 目标句柄，包含要取消的目标信息
            
        返回:
            CancelResponse.ACCEPT: 接受取消请求，execute_callback中is_cancel_requested变为True
            CancelResponse.REJECT: 拒绝取消请求，任务继续执行
        """
        self.node.get_logger().info('取消了请求')
        self.wlkata_arm.senMsg("o117")
        # 接受取消请求（在实际应用中，可以根据情况决定是否接受）
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        执行回调函数
        
        【执行顺序】：第2步（或第3步，如果有handle_accepted_callback）
        【触发时机】：goal_callback返回ACCEPT后，在新线程中异步执行
        【执行线程】：独立工作线程（不会阻塞其他回调）
        【执行特点】：
            - 可以长时间运行（执行实际任务）
            - 在执行过程中需要：
              1. 定期检查 goal_handle.is_cancel_requested（处理取消请求）
              2. 调用 goal_handle.publish_feedback() 发送进度反馈
              3. 任务结束时调用 goal_handle.succeed()/canceled()/abort()
        【与cancel_callback的关系】：
            - cancel_callback可能在execute_callback执行期间被调用
            - execute_callback需要主动检查is_cancel_requested来响应取消
            - 两个回调可能并发执行，需要注意线程安全
        
        参数:
            goal_handle: 目标句柄，包含目标请求和执行控制方法
                - goal_handle.request: 获取原始目标请求
                - goal_handle.is_cancel_requested: 检查是否收到取消请求
                - goal_handle.publish_feedback(): 发送进度反馈
                - goal_handle.succeed(): 标记任务成功完成（不接受参数，结果通过返回值传递）
                - goal_handle.canceled(): 标记任务被取消（不接受参数，结果通过返回值传递）
                - goal_handle.abort(): 标记任务异常终止（不接受参数，结果通过返回值传递）
            
        返回:
            result: 执行结果对象（包含success标志和message）
        """
        # 从目标句柄中获取目标请求
        # 注意：在实际应用中，应该使用goal中的信息（如goal.pose, goal.speed, goal.mode等）
        # 来控制机械臂运动。当前代码只是模拟，所以暂时不使用goal变量
        goal = goal_handle.request
        
        # 在实际应用中，应该根据goal中的参数执行运动，例如：
        # - goal.pose: 目标位姿 [x, y, z, rx, ry, rz, 0]
        # - goal.speed: 运动速度 (0.0-1.0)
        # - goal.mode: 运动模式 (0=点到点, 1=直线, 2=圆弧)
        # - goal.absolute: 是否使用绝对坐标
        # - goal.tool_config: 工具配置（如夹爪状态）
        # 示例：self.robot.move_to(goal.pose, goal.speed, goal.mode)
        
        # 创建反馈对象（用于向客户端发送执行进度）
        feedback = MoveArm.Feedback()
        
        # 创建结果对象（用于返回最终执行结果）
        result = MoveArm.Result()
        
        self.node.get_logger().info('开始控制机械臂')
        
        # ========== 运动执行过程 ==========
        # 在实际应用中，这里应该与硬件通信，控制机械臂运动
        if goal.stop ==True:
            self.wlkata_arm.sendMsg("o117")
        elif goal.restart ==True:
            self.wlkata_arm.restart()
        elif goal.home ==True:
            self.wlkata_arm.homing()
        elif goal.zero ==True:
            self.wlkata_arm.zero()
        elif goal.send_msg !="":
            self.wlkata_arm.sendMsg(goal.send_msg)
        elif goal.file_name !="":
            self.wlkata_arm.runFile(goal.file_name)
        else:
            if goal.pwm_num != -1:
                self.wlkata_arm.pwmWrite(goal.pwm_num)
            elif goal.pump != -1:
                self.wlkata_arm.pump(goal.pump)
            elif goal.gripper != -1:
                self.wlkata_arm.gripper(goal.gripper)

            if goal.speed !=0.0:
                self.wlkata_arm.speed(goal.speed*5400)
            elif goal.speed ==0.0:
                self.wlkata_arm.speed(0.74*5400)

            if goal.mode == 0 or goal.mode == 1 :  # 点到点运动
                self.wlkata_arm.writecoordinate(goal.mode, goal.absolute,goal.pose[0],goal.pose[1],goal.pose[2],goal.pose[3],goal.pose[4],goal.pose[5])
            elif goal.mode == 2:  # 轴角运动
                self.wlkata_arm.writeangle(goal.absolute,goal.pose[0],goal.pose[1],goal.pose[2],goal.pose[3],goal.pose[4],goal.pose[5])
        while True:
            # ========== 检查取消请求 ==========
            # 在执行过程中定期检查是否有取消请求
            # 
            # 【重要说明】：
            # - is_cancel_requested是goal_handle的属性（布尔值）
            # - 当客户端发送取消请求时：
            #   1. cancel_callback会被调用（可能在另一个线程中）
            #   2. 如果cancel_callback返回ACCEPT，is_cancel_requested会变为True
            #   3. execute_callback需要主动检查这个标志来响应取消
            # - 这是execute_callback与cancel_callback之间的通信机制
            # - 两个回调可能并发执行，但通过goal_handle共享状态
            if goal_handle.is_cancel_requested:
                # 如果收到取消请求，设置失败结果并通知客户端
                result.success = False
                # 在ROS2 Humble中，canceled()方法不接受参数，结果通过返回值传递
                goal_handle.canceled()  # 通知客户端任务已取消
                return result
            
            # ========== 更新并发送反馈 ==========
            feedback.status = self.wlkata_arm.getState() # 设置状态信息
            # 向客户端发布反馈（客户端可以实时看到执行进度）
            goal_handle.publish_feedback(feedback)
            
            # 记录当前进度到日志
            self.node.get_logger().info(f'data: {feedback.status}')
            if feedback.status == "Idle":
                break
        
        # ========== 执行完成 ==========
        # 所有步骤完成，设置成功结果
        result.success = True
        # 通知客户端任务成功完成，并返回结果
        # 在ROS2 Humble中，succeed()方法不接受参数，结果通过返回值传递
        goal_handle.succeed()
        self.node.get_logger().info('机械臂运动完成，等待下一指令')
        
        return result
    
    def cleanup(self):
        """
        清理资源：关闭串口
        """
        if hasattr(self, 'sre') and self.sre is not None:
            try:
                if self.sre.is_open:
                    self.sre.close()
                    self.node.get_logger().info('串口已关闭')
            except Exception as e:
                self.node.get_logger().warn(f'关闭串口时出错: {e}')

def main(args=None):
    """
    主函数：初始化ROS2节点，创建Action服务器，并开始监听客户端请求
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建ROS2节点，节点名称为'arm_action_server'
    node = rclpy.create_node('wlkata_mirobot_action_server')
    
    # 创建Action服务器实例
    server = ArmActionServer(node)
    
    # ========== 使用多线程执行器 ==========
    # MultiThreadedExecutor允许多个回调并发执行
    # num_threads=2表示使用2个工作线程
    # 这对于Action服务器很重要，因为可以同时处理多个目标
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)  # 将节点添加到执行器
    
    try:
        # 开始执行器循环，持续监听和处理请求
        # 这是一个阻塞调用，会一直运行直到节点关闭
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt, shutting down...')
    except Exception as e:
        node.get_logger().error(f'Exception in executor: {e}')
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        # 无论是否发生异常，都要清理资源
        node.get_logger().info('Shutting down executor...')
        executor.shutdown()  # 关闭执行器
        # 清理串口资源（如果server已创建）
        if 'server' in locals() and server is not None:
            server.cleanup()
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS2


if __name__ == '__main__':
    main()