#ifndef __DFPLAYER_H
#define __DFPLAYER_H

#include "stm32f1xx_hal.h"

/*
 * ======== DFPlayer Interface ========
 * 这个头文件只保留最小控制接口：
 * 1. 由业务层在初始化时绑定 UART
 * 2. 驱动层只负责拼协议帧和发送命令
 * 3. 后续如果更换为其他串口语音模块，只需要优先修改 dfplayer.c/.h
 */

void DFPlayer_Init(UART_HandleTypeDef *huart);
void DFPlayer_SetVolume(uint8_t volume);
void DFPlayer_PlayMp3(uint16_t index);
void DFPlayer_Pause(void);
void DFPlayer_Resume(void);

#endif /* __DFPLAYER_H */
