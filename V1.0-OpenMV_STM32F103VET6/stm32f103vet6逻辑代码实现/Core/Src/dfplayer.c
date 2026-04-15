#include "dfplayer.h"

/* =========================================================
 * DFPlayer 协议常量
 * =========================================================
 * 这里统一管理 DFPlayer 的固定帧格式与常用命令字。
 * 后续如果更换协议兼容模块，优先修改这里。
 */
#define DFPLAYER_FRAME_START            0x7EU
#define DFPLAYER_FRAME_VERSION          0xFFU
#define DFPLAYER_FRAME_LENGTH           0x06U
#define DFPLAYER_FEEDBACK_DISABLE       0x00U
#define DFPLAYER_FRAME_END              0xEFU
#define DFPLAYER_FRAME_SIZE             10U

#define DFPLAYER_CMD_SET_VOLUME         0x06U
#define DFPLAYER_CMD_RESUME             0x0DU
#define DFPLAYER_CMD_PAUSE              0x0EU
#define DFPLAYER_CMD_PLAY_MP3           0x12U

#define DFPLAYER_VOLUME_MAX             30U
#define DFPLAYER_UART_TIMEOUT_MS        20U   /* 10字节@115200约需0.87ms，20ms留余量且比原100ms减少80%阻塞 */

static UART_HandleTypeDef *dfplayer_huart = NULL;

static uint16_t DFPlayer_GetChecksum(uint8_t command, uint16_t parameter);
static void DFPlayer_SendCommand(uint8_t command, uint16_t parameter);

/*
 * 函数名：DFPlayer_Init
 * 功能：绑定 DFPlayer 使用的 UART 句柄
 *
 * 说明：
 * 驱动层不硬编码 huart1，而是在这里由外部传入，
 * 这样后续更换串口或移植到别的工程时更方便。
 */
void DFPlayer_Init(UART_HandleTypeDef *huart)
{
    dfplayer_huart = huart;
}

/*
 * 函数名：DFPlayer_SetVolume
 * 功能：设置 DFPlayer 音量
 *
 * 参数范围：0~30
 * 如果外部传入超范围值，这里自动做限幅。
 */
void DFPlayer_SetVolume(uint8_t volume)
{
    if (volume > DFPLAYER_VOLUME_MAX)
    {
        volume = DFPLAYER_VOLUME_MAX;
    }

    DFPlayer_SendCommand(DFPLAYER_CMD_SET_VOLUME, volume);
}

/*
 * 函数名：DFPlayer_PlayMp3
 * 功能：播放 MP3 文件夹中的指定编号文件
 *
 * 说明：
 * 例如 index = 1 对应 MP3/0001.mp3
 */
void DFPlayer_PlayMp3(uint16_t index)
{
    if (index == 0U)
    {
        return;
    }

    DFPlayer_SendCommand(DFPLAYER_CMD_PLAY_MP3, index);
}

/*
 * 函数名：DFPlayer_Pause
 * 功能：暂停当前播放
 */
void DFPlayer_Pause(void)
{
    DFPlayer_SendCommand(DFPLAYER_CMD_PAUSE, 0U);
}

/*
 * 函数名：DFPlayer_Resume
 * 功能：继续播放当前音频
 */
void DFPlayer_Resume(void)
{
    DFPlayer_SendCommand(DFPLAYER_CMD_RESUME, 0U);
}

/*
 * 函数名：DFPlayer_GetChecksum
 * 功能：根据 DFPlayer 标准协议计算校验和
 */
static uint16_t DFPlayer_GetChecksum(uint8_t command, uint16_t parameter)
{
    uint16_t sum = 0U;

    sum += DFPLAYER_FRAME_VERSION;
    sum += DFPLAYER_FRAME_LENGTH;
    sum += command;
    sum += DFPLAYER_FEEDBACK_DISABLE;
    sum += (uint8_t)(parameter >> 8);
    sum += (uint8_t)(parameter & 0x00FFU);

    return (uint16_t)(0U - sum);
}

/*
 * 函数名：DFPlayer_SendCommand
 * 功能：按标准 10 字节协议帧向 DFPlayer 发送命令
 *
 * 说明：
 * 1. 如果尚未绑定 UART，直接返回，避免业务层卡死
 * 2. 当前版本使用阻塞发送，逻辑简单、可控，适合本工程
 */
static void DFPlayer_SendCommand(uint8_t command, uint16_t parameter)
{
    uint8_t frame[DFPLAYER_FRAME_SIZE];
    uint16_t checksum;

    if (dfplayer_huart == NULL)
    {
        return;
    }

    checksum = DFPlayer_GetChecksum(command, parameter);

    frame[0] = DFPLAYER_FRAME_START;
    frame[1] = DFPLAYER_FRAME_VERSION;
    frame[2] = DFPLAYER_FRAME_LENGTH;
    frame[3] = command;
    frame[4] = DFPLAYER_FEEDBACK_DISABLE;
    frame[5] = (uint8_t)(parameter >> 8);
    frame[6] = (uint8_t)(parameter & 0x00FFU);
    frame[7] = (uint8_t)(checksum >> 8);
    frame[8] = (uint8_t)(checksum & 0x00FFU);
    frame[9] = DFPLAYER_FRAME_END;

    HAL_UART_Transmit(dfplayer_huart, frame, DFPLAYER_FRAME_SIZE, DFPLAYER_UART_TIMEOUT_MS);
}
