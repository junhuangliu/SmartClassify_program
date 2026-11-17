/*
 * uart_echo_test.h
 *
 * 串口自测用的小函数（临时调试，正式使用可以删掉）。
 *
 * 作用：树莓派通过 CH340 发一行文字给 STM32，
 *        STM32 原样回一行，证明“线通了、波特率对了”。
 *
 * 用法：在 uart1_proto.c 的 Uart1Proto_ProcessLine 开头调用一次，
 *       返回 1 表示这行已经被这里处理掉了，不用再往下走。
 */

#ifndef __UART_ECHO_TEST_H
#define __UART_ECHO_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 处理一行文本：
   收到 $GRAB... 或 $NONE  -> 回 "$ECHO,<原内容>"
   收到 ECHO 开头的文字    -> 回 "OK ECHO,<后面的文字>"
   其它                    -> 返回 0，让原来的协议继续处理
   返回 1 表示已经回复过了 */
int UartEchoTest_ProcessLine(const char *line);

#ifdef __cplusplus
}
#endif

#endif /* __UART_ECHO_TEST_H */
