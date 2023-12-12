#include "common_inc.h"

extern DummyRobot dummy;


/**
 * Processes a USB ASCII command.
 *
 * @param _cmd The command to process.
 * @param _len The length of the command.
 * @param _responseChannel The channel to send the response to.
 *
 * @throws ErrorType Description of any possible error that could occur during processing.
 */
/**
 * 处理USB ASCII命令。
 *
 * @param _cmd 要处理的命令。
 * @param _len 命令的长度。
 * @param _responseChannel 响应通道。
 *
 * @throws ErrorType 在处理过程中可能发生的任何错误的描述。
 */
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    uint8_t  i; // 定义一个8位的无符号整型变量i，但在此代码段中未使用

    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    // 判断接收到的命令类型并执行相应的操作

    if (_cmd[0] == '!' || !dummy.IsEnabled()) // 如果命令以'!'开始，或者dummy设备未启用
    {
        std::string s(_cmd); // 将命令转换为std::string类型，便于后续处理

        // 判断命令内容并执行相应的操作
        if (s.find("STOP") != std::string::npos) // 如果命令包含"STOP"
        {
            dummy.commandHandler.EmergencyStop(); // 调用dummy的紧急停止函数
            Respond(_responseChannel, "Stopped ok"); // 向响应通道发送停止成功的消息
        } else if (s.find("START") != std::string::npos) // 如果命令包含"START"
        {
            dummy.SetEnable(true); // 启用dummy设备
            Respond(_responseChannel, "Started ok"); // 向响应通道发送启动成功的消息
        } else if (s.find("HOME") != std::string::npos) // 如果命令包含"HOME"
        {
            dummy.Homing(); // 调用dummy的归位函数
            Respond(_responseChannel, "Started ok"); // 向响应通道发送归位成功的消息
        } else if (s.find("RESET") != std::string::npos) // 如果命令包含"RESET"
        {
            dummy.Resting(); // 调用dummy的复位函数
            Respond(_responseChannel, "Started ok"); // 向响应通道发送复位成功的消息
        } else if (s.find("DISABLE") != std::string::npos) // 如果命令包含"DISABLE"
        {
            dummy.SetEnable(false); // 禁用dummy设备
            Respond(_responseChannel, "Disabled ok"); // 向响应通道发送禁用成功的消息
        }
    } else if (_cmd[0] == '#') // 如果命令以'#'开头
    {
        std::string s(_cmd); // 将命令转换为std::string类型

        // 判断命令内容并执行相应的操作
        if (s.find("GETJPOS") != std::string::npos) // 如果命令包含"GETJPOS"
        {
            // 响应当前关节位置信息
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos) // 如果命令包含"GETLPOS"
        {
            dummy.UpdateJointPose6D(); // 更新6D姿态信息
            // 响应当前6D姿态信息
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos) // 如果命令包含"CMDMODE"
        {
            uint32_t mode; // 定义一个32位无符号整型变量mode
            sscanf(_cmd, "#CMDMODE %lu", &mode); // 从命令中解析出模式值
            dummy.SetCommandMode(mode); // 设置命令模式
            Respond(_responseChannel, "Set command mode to [%lu]", mode); // 向响应通道发送设置命令模式成功的消息
        } else
            Respond(_responseChannel, "ok"); // 如果是其他'#'开头的命令，回复"ok"
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&') // 如果命令以'>', '@', '&'中的任意一个开始
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd); // 将命令添加到处理队列中，并获取剩余空间大小
        Respond(_responseChannel, "%d", freeSize); // 向响应通道发送剩余空间大小
    }

    /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}



/**
 * Handles UART ASCII commands.
 *
 * @param _cmd pointer to the command string
 * @param _len length of the command string
 * @param _responseChannel the channel to send responses to
 *
 * @throws ErrorType if there is an error in command handling
 */
/**
 * 处理UART ASCII命令。
 *
 * @param _cmd 指向命令字符串的指针
 * @param _len 命令字符串的长度
 * @param _responseChannel 用于发送响应的通道
 *
 * @throws ErrorType 如果命令处理出错
 */
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("HOME") != std::string::npos)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("RESET") != std::string::npos)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


/**
 * Handles UART5 ASCII commands.
 *
 * @param _cmd The command string.
 * @param _len The length of the command string.
 * @param _responseChannel The channel to send the response to.
 *
 * @throws ErrorType Description of any potential errors.
 */
/**
 * 处理 UART5 ASCII 命令。
 *
 * @param _cmd 命令字符串。
 * @param _len 命令字符串长度。
 * @param _responseChannel 响应通道。
 *
 * @throws ErrorType 潜在错误的描述。
 */
void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}