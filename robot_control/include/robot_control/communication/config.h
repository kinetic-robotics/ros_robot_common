#ifndef ROBOT_CONTROL_COMMUNICATION_CONFIG_H_
#define ROBOT_CONTROL_COMMUNICATION_CONFIG_H_

/* 通信SOF */
#define PKG_SOF 0xAA

/* 通信指令码 */
#define CMD_CAN1_RX 1
#define CMD_CAN2_RX 2
#define CMD_UART1_RX 3
#define CMD_UART2_RX 4
#define CMD_UART6_RX 5
#define CMD_UART7_RX 6
#define CMD_UART8_RX 7
#define CMD_CAN1_TX 10
#define CMD_CAN2_TX 11
#define CMD_UART1_TX 12
#define CMD_UART2_TX 13
#define CMD_UART6_TX 14
#define CMD_UART7_TX 15
#define CMD_UART8_TX 16
#define CMD_ERROR 100

/* 出错设备ID */
#define ERROR_CAN1 1
#define ERROR_CAN2 2
#define ERROR_UART1 3
#define ERROR_UART2 4
#define ERROR_UART6 5
#define ERROR_UART7 6
#define ERROR_UART8 7

/* CAN错误 */
#define CAN_ERROR_EWG 0x00000001U             /* Protocol Error Warning */
#define CAN_ERROR_EPV 0x00000002U             /* Error Passive */
#define CAN_ERROR_BOF 0x00000004U             /* Bus-off error */
#define CAN_ERROR_STF 0x00000008U             /* Stuff error */
#define CAN_ERROR_FOR 0x00000010U             /* Form error */
#define CAN_ERROR_ACK 0x00000020U             /* Acknowledgment error */
#define CAN_ERROR_BR 0x00000040U              /* Bit recessive error */
#define CAN_ERROR_BD 0x00000080U              /* Bit dominant error */
#define CAN_ERROR_CRC 0x00000100U             /* CRC error */
#define CAN_ERROR_RX_FOV0 0x00000200U         /* Rx FIFO0 overrun error */
#define CAN_ERROR_RX_FOV1 0x00000400U         /* Rx FIFO1 overrun error */
#define CAN_ERROR_TX_ALST0 0x00000800U        /* TxMailbox 0 transmit failure due to arbitration lost */
#define CAN_ERROR_TX_TERR0 0x00001000U        /* TxMailbox 1 transmit failure due to tranmit error */
#define CAN_ERROR_TX_ALST1 0x00002000U        /* TxMailbox 0 transmit failure due to arbitration lost */
#define CAN_ERROR_TX_TERR1 0x00004000U        /* TxMailbox 1 transmit failure due to tranmit error */
#define CAN_ERROR_TX_ALST2 0x00008000U        /* TxMailbox 0 transmit failure due to arbitration lost */
#define CAN_ERROR_TX_TERR2 0x00010000U        /* TxMailbox 1 transmit failure due to tranmit error */
#define CAN_ERROR_TIMEOUT 0x00020000U         /* Timeout error */
#define CAN_ERROR_NOT_INITIALIZED 0x00040000U /* Peripheral not initialized */
#define CAN_ERROR_NOT_READY 0x00080000U       /* Peripheral not ready */
#define CAN_ERROR_NOT_STARTED 0x00100000U     /* Peripheral not started */
#define CAN_ERROR_PARAM 0x00200000U           /* Parameter error */
#define CAN_ERROR_INTERNAL 0x00800000U        /* Internal error */
#define CAN_ERROR_SEND_TIMEOUT 0x01000000U    /* Send timeout error */
#define CAN_ERROR_COMMUNICATE_TIMEOUT 0x10000000U /* Forwarding timeout error. */

/* UART错误 */
#define UART_ERROR_PE 0x00000001U  /* Parity error */
#define UART_ERROR_NE 0x00000002U  /* Noise error */
#define UART_ERROR_FE 0x00000004U  /* Frame error */
#define UART_ERROR_ORE 0x00000008U /* Overrun error */
#define UART_ERROR_DMA 0x00000010U /* DMA transfer error  */

/* CAN口数量,用于分配ID */
#define CAN_NUMBER 2

/* SERIAL口数量,用于分配ID */
#define SERIAL_NUMBER 5

#endif