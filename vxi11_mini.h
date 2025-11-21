/********************************************************************************


 **** Copyright (C), 2025, <YourName>
 **** All rights reserved

 ********************************************************************************
 * File Name     : vxi11_mini.h
 * Author        : <YourName>
 * Date          : 2025-11-21
 * Version       : 1.2
********************************************************************************/
/**************************************************************************/
/*
    最简 VXI-11 Core 封装库 (单头+单源)
        - 仅支持 create_link(10), device_write(11), device_read(12)
        - 固定 Program=0x000607AF, Version=1
        - 不使用 rpcbind/portmapper
        - 不支持分片 (Record Marker 高位必须为 1)
        - 单 Link (ID=0)
        - AUTH_NULL
        - 通过“port接口”进行收发：Getc(单字节取)、Send(多字节发)
        - 默认绑定“虚拟端口”，移植时用 vxi11_bind_port 绑定到实际物理接口

    典型使用:
        vxi11_init();
        vxi11_bind_port(SerialSendPort, SerialGetcPort); // 移植时绑定你的物理接口
        while (1) {
            vxi11_task();  // 从port读字节->组包->解析->通过port发送REPLY
            // 通过 vxi11_pop_scpi_input() 取到SCPI命令，解析后
            // vxi11_set_scpi_response() 填入待发数据
        }

    虚拟端口辅助:
        - vxi11_vport_inject_rx(): 向虚拟RX注入原始网络字节(客户端->设备)
        - vxi11_vport_drain_tx():  从虚拟TX取走已准备发送的字节(设备->客户端)

*/
/**************************************************************************/

#ifndef _VXI11_MINI_H_
#define _VXI11_MINI_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* Exported constants / macros ----------------------------------------------*/
/* 缓冲区大小（可在编译前重定义） */
#ifndef VXI11_RX_PORT_BUFFER_SIZE
#define VXI11_RX_PORT_BUFFER_SIZE (2048U) /* 内部组包RX缓冲（RPC层） */
#endif
#ifndef VXI11_SCPI_IN_BUFFER_SIZE
#define VXI11_SCPI_IN_BUFFER_SIZE (1024U) /* 待解析SCPI输入缓冲 */
#endif
#ifndef VXI11_SCPI_OUT_BUFFER_SIZE
#define VXI11_SCPI_OUT_BUFFER_SIZE (1024U) /* 待返回SCPI输出缓冲 */
#endif

/* 虚拟端口的环形缓冲容量（仅默认端口使用） */
#ifndef VXI11_VPORT_RX_SIZE
#define VXI11_VPORT_RX_SIZE (4096U) /* 虚拟接收缓冲：模拟“来自客户端”的网络字节 */
#endif
#ifndef VXI11_VPORT_TX_SIZE
#define VXI11_VPORT_TX_SIZE (4096U) /* 虚拟发送缓冲：模拟“发往客户端”的网络字节 */
#endif

/* 库内部通用返回 */
#define VXI11_RET_OK  (0U)
#define VXI11_RET_ERR (1U)

/* port返回（参考modbusX风格） */
#define VXI11_PORT_RETURN_DEFAULT   (0U)
#define VXI11_PORT_RETURN_ERR_INDEF (1U)

/* VXI-11 常量 */
#define VXI11_PROGRAM_CORE      (0x000607AFu)
#define VXI11_VERSION_CORE      (0x00000001u)

#define VXI11_PROC_CREATE_LINK  (10U)
#define VXI11_PROC_DEVICE_WRITE (11U)
#define VXI11_PROC_DEVICE_READ  (12U)

/* RPC 通用 */
#define VXI11_RPC_MSGTYPE_CALL     (0U)
#define VXI11_RPC_MSGTYPE_REPLY    (1U)
#define VXI11_REPLY_MSG_ACCEPTED   (0U)
#define VXI11_REPLY_ACCEPT_SUCCESS (0U)

/* device_read reason (简化) */
#define VXI11_READ_REASON_END     (0x00000004u) /* 全部发送完毕 */
#define VXI11_READ_REASON_PARTIAL (0x00000002u) /* 仅部分发送，仍有余量 */

/* Exported types ------------------------------------------------------------*/
/* 与modbusX风格一致的端口函数原型 */
typedef uint32_t (*VXI11_SEND_PTR)(const void *Data, size_t Len);
typedef uint32_t (*VXI11_GTEC_PTR)(uint8_t *Data);

/**
 * @brief 运行时上下文
 */
typedef struct
{
    uint8_t  LinkCreated;   /* 是否已建立link */
    uint32_t RxBytes;       /* 内部RPC-RX缓冲已用 */
    uint32_t ScpiInBytes;   /* SCPI输入缓冲已用 */
    uint32_t ScpiOutBytes;  /* SCPI输出缓冲已用 */
    uint32_t LastErrorCode; /* 最近内部错误码(库内记录) */
    uint32_t LinkID;        /* 固定0 */
} _VXI11_CTX;

/* Exported variables --------------------------------------------------------*/
const _VXI11_CTX *vxi11_get_ctx(void);

/* Exported functions --------------------------------------------------------*/
/* 初始化库（清空状态与缓冲，绑定默认虚拟端口） */
void vxi11_init(void);

/* 绑定外部物理端口函数（NULL则沿用默认虚拟端口） */
void vxi11_bind_port(VXI11_SEND_PTR send_cb, VXI11_GTEC_PTR getc_cb);

/* 周期驱动：从port取字节→组包→解析→通过port发送REPLY */
void vxi11_task(void);

/* 取走SCPI输入缓冲供外部解析 */
uint32_t vxi11_pop_scpi_input(uint8_t *out, size_t max_len, size_t *actual_len);

/* 查询SCPI输入缓冲长度 */
size_t vxi11_peek_scpi_input_len(void);

/* 外部将SCPI解析结果作为回复写入库 */
uint32_t vxi11_set_scpi_response(const uint8_t *data, size_t len);

/* 查询SCPI回复缓冲长度 */
size_t vxi11_scpi_response_size(void);

/* 清空SCPI输入/输出缓冲 */
void vxi11_clear_scpi_input(void);
void vxi11_clear_scpi_response(void);

/* -------------------- 虚拟端口辅助API（可选） -------------------- */
/* 注入“客户端->设备”的原始网络字节到虚拟RX缓冲（供Getc读取） */
uint32_t vxi11_vport_inject_rx(const uint8_t *data, size_t len);
/* 从虚拟TX缓冲取走“设备->客户端”的字节 */
uint32_t vxi11_vport_drain_tx(uint8_t *out, size_t max_len, size_t *actual_len);
/* 查询虚拟TX/RX当前长度（调试辅助） */
size_t vxi11_vport_peek_tx_len(void);
size_t vxi11_vport_peek_rx_len(void);

#ifdef __cplusplus
}
#endif
#endif /* _VXI11_MINI_H_ */