"""
ROS2 Action 客户端 - 机械臂控制客户端
功能：向机械臂Action服务器发送运动指令，并接收执行结果
"""

import rclpy
from rclpy.action import ActionClient  # Action客户端
from action_msgs.msg import GoalStatus  # 目标状态枚举（从action_msgs导入）

from wlkata_interfaces.action import MoveArm  # 机械臂运动Action接口
from wlkata_interfaces.msg import ToolConfig  # 工具配置消息类型
import sys 
def main(args=None):
    """
    主函数：初始化ROS2节点，创建Action客户端，发送运动指令并等待结果
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建ROS2节点，节点名称为'arm_action_client'
    node = rclpy.create_node('wlkata_mirobot_action_client')
    
    # 创建Action客户端
    # 参数：节点对象、Action类型、Action名称（必须与服务器端一致）
    client = ActionClient(node, MoveArm, 'move_arm')
    
    # 等待Action服务器上线
    # 这是一个阻塞调用，直到服务器可用才会继续执行
    node.get_logger().info('Waiting for action server...')
    client.wait_for_server()
    
    # ========== 构建目标指令 (Goal) ==========
    goal = MoveArm.Goal()  # 创建运动目标对象
    
    # 设置运动模式：0表示绝对坐标，1表示相对坐标
    goal.absolute = 0
    
    # 设置运动模式：0=点到点运动，1=直线运动，2=轴角运动
    goal.mode = 0  # 0=点到点
    
    # 设置目标位姿：[x, y, z, rx, ry, rz, 0]
    # x, y, z: 位置坐标（米）
    # rx, ry, rz: 旋转角度（弧度）或四元数
    # 最后一个参数通常为0或保留
    # 从命令行参数获取z坐标值（需要转换为float类型）
    try:
        if len(sys.argv) < 2:
            node.get_logger().error('缺少命令行参数：请提供z坐标值')
            node.get_logger().info('用法: ros2 run mirobot_uart mirobot_client1 <z坐标>')
            node.destroy_node()
            rclpy.shutdown()
            return
        z_coordinate = float(sys.argv[1])  # 将字符串转换为浮点数
    except ValueError:
        node.get_logger().error(f'无效的参数值: {sys.argv[1]}，必须是数字')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    goal.pose = [257.5, 0.0, z_coordinate, 0.0, 0.0, 0.0, 0.0]
    
    # 设置运动速度：范围0.0-1.0，1.0为最大速度
    # 如果提供了第二个命令行参数，使用它作为速度；否则使用默认值0.7
    if len(sys.argv) >= 3:
        try:
            goal.speed = float(sys.argv[2])
            if not (0.0 <= goal.speed <= 1.0):
                node.get_logger().warn(f'速度值 {goal.speed} 超出范围 [0.0, 1.0]，使用默认值 0.7')
                goal.speed = 0.7
        except ValueError:
            node.get_logger().warn(f'无效的速度值: {sys.argv[2]}，使用默认值 0.7')
            goal.speed = 0.7
    else:
        goal.speed = 0.7  # 默认速度
    
    
    # ========== 发送目标到服务器 ==========
    node.get_logger().info('Sending goal...')
    # 异步发送目标，返回一个Future对象（用于异步等待结果）
    send_goal_future = client.send_goal_async(goal)
    
    # 等待服务器响应（接受或拒绝目标）
    # 使用循环方式等待Future完成，并实现超时机制
    node.get_logger().info('Waiting for server response...')
    timeout_seconds = 10.0
    start_time = node.get_clock().now()
    
    # 先检查是否已经完成（可能服务器响应很快）
    if not send_goal_future.done():
        # 如果未完成，循环等待
        while rclpy.ok():
            # 检查是否超时
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_seconds:
                node.get_logger().error(f'Timeout waiting for server response ({timeout_seconds}s)')
                node.destroy_node()
                rclpy.shutdown()
                return
            
            # 处理一次回调（非阻塞）
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 检查Future是否完成
            if send_goal_future.done():
                break
    
    # 获取目标句柄（goal_handle），包含服务器对目标的响应信息
    try:
        goal_handle = send_goal_future.result()
    except Exception as e:
        node.get_logger().error(f'Failed to get goal handle: {e}')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 检查是否成功获取目标句柄
    if goal_handle is None:
        node.get_logger().error('Failed to get goal handle')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 调用目标响应回调函数，处理服务器对目标的响应
    goal_response_callback(goal_handle)
    
    # 检查目标是否被服务器接受
    # 如果被拒绝，则退出程序
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    node.get_logger().info('Goal accepted')
    
    # ========== 等待执行结果 ==========
    # 获取结果Future，用于异步等待任务执行完成
    result_future = goal_handle.get_result_async()
    
    # 循环等待结果完成
    # 在等待过程中，ROS2会自动处理来自服务器的反馈消息
    node.get_logger().info('Waiting for result...')
    timeout_count = 0
    max_timeout = 300  # 最大等待时间：300 * 0.1 = 30秒
    while rclpy.ok():  # rclpy.ok()检查ROS2是否仍在运行
        # 处理一次回调（非阻塞，超时0.1秒）
        # 这允许节点处理接收到的消息和回调
        rclpy.spin_once(node, timeout_sec=0.1)
        
        # 注意：反馈消息通过ROS2的订阅机制自动接收
        # 如果需要处理反馈，可以订阅反馈主题或使用其他机制
        
        # 检查结果Future是否已完成
        if result_future.done():
            try:
                # 获取执行结果响应对象
                # 注意：result_future.result()返回的是GetResult_Response对象
                # 实际的结果对象在response.result中
                response = result_future.result()
                # 从响应对象中提取实际的结果对象
                result = response.result
                # 调用结果回调函数处理结果（传递node对象用于日志记录）
                result_callback(goal_handle, result, node)
            except Exception as e:
                node.get_logger().error(f'Error getting result: {e}')
            break  # 结果已收到，退出循环
        
        # 超时检查
        timeout_count += 1
        if timeout_count >= max_timeout:
            node.get_logger().error(f'Timeout waiting for result (>{max_timeout * 0.1} seconds)')
            break
    
    # 清理资源：销毁节点并关闭ROS2
    node.destroy_node()
    rclpy.shutdown()

def goal_response_callback(goal_handle):
    """
    目标响应回调函数
    当服务器接受或拒绝目标时调用
    
    参数:
        goal_handle: 目标句柄，包含accept/reject信息
    """
    if goal_handle.accepted:
        print('Goal accepted')  # 目标被接受
    else:
        print('Goal rejected')  # 目标被拒绝

def feedback_callback(feedback_msg):
    """
    反馈回调函数（当前未使用，保留以备将来扩展）
    当服务器发送执行进度反馈时调用
    
    参数:
        feedback_msg: 反馈消息对象，包含进度和状态信息
    """
    feedback = feedback_msg.feedback
    # 打印进度百分比和状态
    print(f'状态：{feedback.status}')

def result_callback(goal_handle, result, node=None):
    """
    结果回调函数
    当任务执行完成（成功或失败）时调用
    
    参数:
        goal_handle: 目标句柄，包含执行状态
        result: 执行结果对象（MoveArm.Result），包含成功标志和消息
        node: ROS2节点对象（可选，用于日志记录）
    """
    # 获取目标执行状态
    status = goal_handle.status
    
    # 检查是否执行成功
    # GoalStatus.STATUS_SUCCEEDED = 4 (成功状态码)
    # 如果GoalStatus导入失败，可以直接使用状态码: status == 4
    if status == GoalStatus.STATUS_SUCCEEDED:
        # 使用节点logger记录日志（如果提供了node对象）
        if node:
            node.get_logger().info('Goal succeeded')
        print('Goal succeeded')
        # 如果有结果消息，打印出来
        # 注意：result对象的结构取决于MoveArm.Result的定义
        # 如果result有message属性，则打印；否则打印整个result对象
        if result:
            if hasattr(result, 'message'):
                print(f'Result: {result.message}')
            elif hasattr(result, 'success'):
                print(f'Result: success={result.success}')
            else:
                print(f'Result: {result}')
    else:
        # 执行失败或其他状态
        if node:
            node.get_logger().warn(f'Goal failed with status: {status}')
        print(f'Goal failed with status: {status}')
        # 如果有结果，也打印出来
        if result:
            if hasattr(result, 'message'):
                print(f'Result message: {result.message}')
            elif hasattr(result, 'success'):
                print(f'Result success: {result.success}')

if __name__ == '__main__':
    main()