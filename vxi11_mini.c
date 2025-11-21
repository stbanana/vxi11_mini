/********************************************************************************


 **** Copyright (C), 2025, <YourName>
 **** All rights reserved

 ********************************************************************************
 * File Name     : vxi11_mini.c
 * Author        : <YourName>
 * Date          : 2025-11-21
 * Version       : 1.2
********************************************************************************/
/**************************************************************************/
/*
    最简 VXI-11 Core 源文件
        - 单链接、单片Record、AUTH_NULL
        - 通过“port”接口进行单字节接收、多字节发送
        - 默认绑定“虚拟端口”（环形缓冲模拟接口）
*/
/**************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "vxi11_mini.h"
#include <string.h>

/* Private types -------------------------------------------------------------*/
typedef struct
{
    uint8_t *Buf;
    size_t   Cap;
    size_t   Len;
    size_t   Head;
    size_t   Tail;
} _RING;

/* Private macros ------------------------------------------------------------*/
#define VXI11_INLINE static inline

/* 私有错误码（库内记录使用，可扩展到回复error字段） */
#define VXI11_ERRCODE_NONE             (0U)
#define VXI11_ERRCODE_BAD_CALL         (1U)
#define VXI11_ERRCODE_OVERFLOW         (2U)
#define VXI11_ERRCODE_LINK_NOT_CREATED (3U)

/* Private variables ---------------------------------------------------------*/
/* 端口回调（默认绑定到虚拟端口） */
static VXI11_SEND_PTR s_send_cb = NULL;
static VXI11_GTEC_PTR s_getc_cb = NULL;

/* 虚拟端口环形缓冲 */
static uint8_t s_vrx_mem[VXI11_VPORT_RX_SIZE];
static uint8_t s_vtx_mem[VXI11_VPORT_TX_SIZE];
static _RING   s_vrx = {s_vrx_mem, VXI11_VPORT_RX_SIZE, 0, 0, 0};
static _RING   s_vtx = {s_vtx_mem, VXI11_VPORT_TX_SIZE, 0, 0, 0};

/* 内部RPC层组包缓冲（线性） */
static uint8_t s_rx_buf[VXI11_RX_PORT_BUFFER_SIZE];
static size_t  s_rx_len = 0;

/* SCPI收发缓冲 */
static uint8_t s_scpi_in[VXI11_SCPI_IN_BUFFER_SIZE];
static size_t  s_scpi_in_len = 0;

static uint8_t s_scpi_out[VXI11_SCPI_OUT_BUFFER_SIZE];
static size_t  s_scpi_out_len = 0;

/* 上下文 */
static _VXI11_CTX s_ctx;

/* Private function prototypes -----------------------------------------------*/
VXI11_INLINE uint32_t be32_rd(const uint8_t *p);
VXI11_INLINE void     be32_wr(uint8_t *p, uint32_t v);

static uint32_t       ring_push(_RING *r, const uint8_t *data, size_t len);
static uint32_t       ring_pop_byte(_RING *r, uint8_t *out);
static size_t         ring_peek_len(const _RING *r);
static void           ring_clear(_RING *r);

static uint32_t       vport_send(const void *Data, size_t Len);
static uint32_t       vport_getc(uint8_t *Data);

static uint32_t       rx_peek_record(size_t *total_len);
static void           rx_consume(size_t n);

static uint32_t       build_reply_common(uint8_t *dst, uint32_t xid);
static void           finalize_record_marker(uint8_t *dst, size_t payload_len);

static void           process_call(const uint8_t *record, size_t record_len);
static uint32_t       handle_create_link(const uint8_t *payload, size_t payload_len, uint32_t xid);
static uint32_t       handle_device_write(const uint8_t *payload, size_t payload_len, uint32_t xid);
static uint32_t       handle_device_read(const uint8_t *payload, size_t payload_len, uint32_t xid);

static void           pull_from_port_into_rxbuf(void);

/* Private functions ---------------------------------------------------------*/
VXI11_INLINE uint32_t be32_rd(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
VXI11_INLINE void be32_wr(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

/* --- 虚拟端口环形缓冲 --- */
static uint32_t ring_push(_RING *r, const uint8_t *data, size_t len)
{
    if(r->Len + len > r->Cap)
        return VXI11_RET_ERR;
    for(size_t i = 0; i < len; ++i)
    {
        r->Buf[r->Tail] = data[i];
        r->Tail         = (r->Tail + 1) % r->Cap;
    }
    r->Len += len;
    return VXI11_RET_OK;
}
static uint32_t ring_pop_byte(_RING *r, uint8_t *out)
{
    if(r->Len == 0)
        return VXI11_RET_ERR;
    *out    = r->Buf[r->Head];
    r->Head = (r->Head + 1) % r->Cap;
    r->Len--;
    return VXI11_RET_OK;
}
static size_t ring_peek_len(const _RING *r)
{
    return r->Len;
}
static void ring_clear(_RING *r)
{
    r->Len  = 0;
    r->Head = 0;
    r->Tail = 0;
}

/* --- 默认虚拟port实现：Send(多字节)、Getc(单字节) --- */
static uint32_t vport_send(const void *Data, size_t Len)
{
    if(!Data || Len == 0)
        return VXI11_PORT_RETURN_DEFAULT;
    return (ring_push(&s_vtx, (const uint8_t *)Data, Len) == VXI11_RET_OK) ? VXI11_PORT_RETURN_DEFAULT : VXI11_PORT_RETURN_ERR_INDEF;
}
static uint32_t vport_getc(uint8_t *Data)
{
    if(!Data)
        return VXI11_PORT_RETURN_ERR_INDEF;
    return (ring_pop_byte(&s_vrx, Data) == VXI11_RET_OK) ? VXI11_PORT_RETURN_DEFAULT : VXI11_PORT_RETURN_ERR_INDEF;
}

/* --- 组包辅助 --- */
static uint32_t rx_peek_record(size_t *total_len)
{
    if(s_rx_len < 4)
        return VXI11_RET_ERR;
    uint32_t marker = be32_rd(s_rx_buf);
    uint32_t last   = marker & 0x80000000u;
    uint32_t frag   = marker & 0x7FFFFFFFu;
    if(!last)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return VXI11_RET_ERR;
    }
    if(s_rx_len < (size_t)(4 + frag))
        return VXI11_RET_ERR;
    *total_len = 4 + frag;
    return VXI11_RET_OK;
}
static void rx_consume(size_t n)
{
    if(n >= s_rx_len)
        s_rx_len = 0;
    else
    {
        memmove(s_rx_buf, s_rx_buf + n, s_rx_len - n);
        s_rx_len -= n;
    }
    s_ctx.RxBytes = (uint32_t)s_rx_len;
}

/* --- 构包辅助 --- */
static uint32_t build_reply_common(uint8_t *dst, uint32_t xid)
{
    be32_wr(dst + 4, xid);
    be32_wr(dst + 8, VXI11_RPC_MSGTYPE_REPLY);
    be32_wr(dst + 12, VXI11_REPLY_MSG_ACCEPTED);
    be32_wr(dst + 16, 0);
    be32_wr(dst + 20, 0);
    be32_wr(dst + 24, VXI11_REPLY_ACCEPT_SUCCESS);
    be32_wr(dst + 28, VXI11_ERRCODE_NONE);
    return 32U;
}
static void finalize_record_marker(uint8_t *dst, size_t payload_len)
{
    uint32_t marker = 0x80000000u | (uint32_t)payload_len;
    be32_wr(dst, marker);
}

/* --- 从port拉取尽可能多的字节进入内部RPC-RX缓冲 --- */
static void pull_from_port_into_rxbuf(void)
{
    uint8_t ch;
    while(s_getc_cb && s_rx_len < VXI11_RX_PORT_BUFFER_SIZE)
    {
        uint32_t pr = s_getc_cb(&ch);
        if(pr != VXI11_PORT_RETURN_DEFAULT)
            break; /* 无字节可取 */
        s_rx_buf[s_rx_len++] = ch;
    }
    s_ctx.RxBytes = (uint32_t)s_rx_len;
}

/* --- 处理单个CALL --- */
static void process_call(const uint8_t *record, size_t record_len)
{
    if(record_len < 44)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return;
    } /* 4+40 */

    const uint8_t *p         = record + 4;
    size_t         payload   = record_len - 4;

    uint32_t       xid       = be32_rd(p + 0);
    uint32_t       mtype     = be32_rd(p + 4);
    uint32_t       rpcvers   = be32_rd(p + 8);
    uint32_t       program   = be32_rd(p + 12);
    uint32_t       version   = be32_rd(p + 16);
    uint32_t       procedure = be32_rd(p + 20);

    if(mtype != VXI11_RPC_MSGTYPE_CALL || rpcvers != 2 || program != VXI11_PROGRAM_CORE || version != VXI11_VERSION_CORE)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return;
    }

    switch(procedure)
    {
    case VXI11_PROC_CREATE_LINK:
        (void)handle_create_link(p, payload, xid);
        break;
    case VXI11_PROC_DEVICE_WRITE:
        if(!s_ctx.LinkCreated)
        {
            s_ctx.LastErrorCode = VXI11_ERRCODE_LINK_NOT_CREATED;
            return;
        }
        (void)handle_device_write(p, payload, xid);
        break;
    case VXI11_PROC_DEVICE_READ:
        if(!s_ctx.LinkCreated)
        {
            s_ctx.LastErrorCode = VXI11_ERRCODE_LINK_NOT_CREATED;
            return;
        }
        (void)handle_device_read(p, payload, xid);
        break;
    default:
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        break;
    }
}

/* --- create_link --- */
static uint32_t handle_create_link(const uint8_t *payload, size_t payload_len, uint32_t xid)
{
    (void)payload;
    (void)payload_len;
    s_ctx.LinkCreated = 1;
    s_ctx.LinkID      = 0;

    uint8_t  frame[64];
    uint32_t hdr = build_reply_common(frame, xid);
    be32_wr(frame + 36, 0);   /* linkID */
    be32_wr(frame + 40, 0);   /* abortPort 简化 */
    be32_wr(frame + 44, 512); /* maxRecvSize 示例 */

    size_t pay = hdr + 16; /* 32 + 16 = 48 */
    finalize_record_marker(frame, pay);
    /* 发送 */
    return (s_send_cb(frame, pay + 4) == VXI11_PORT_RETURN_DEFAULT) ? VXI11_RET_OK : VXI11_RET_ERR;
}

/* --- device_write --- */
static uint32_t handle_device_write(const uint8_t *payload, size_t payload_len, uint32_t xid)
{
    if(payload_len < 60)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return VXI11_RET_ERR;
    }

    uint32_t data_len = be32_rd(payload + 56);
    if(payload_len < 60 + data_len)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return VXI11_RET_ERR;
    }

    if(data_len > 0)
    {
        size_t can = data_len;
        if(can + s_scpi_in_len > VXI11_SCPI_IN_BUFFER_SIZE)
        {
            can                 = VXI11_SCPI_IN_BUFFER_SIZE - s_scpi_in_len;
            s_ctx.LastErrorCode = VXI11_ERRCODE_OVERFLOW;
        }
        if(can > 0)
        {
            memcpy(&s_scpi_in[s_scpi_in_len], payload + 60, can);
            s_scpi_in_len += can;
            s_ctx.ScpiInBytes = (uint32_t)s_scpi_in_len;
        }
    }

    uint8_t  frame[48];
    uint32_t hdr = build_reply_common(frame, xid);
    be32_wr(frame + 32, VXI11_ERRCODE_NONE);
    be32_wr(frame + 36, data_len); /* 回显原请求长度 */

    size_t pay = hdr + 8; /* 32 + 8 = 40 */
    finalize_record_marker(frame, pay);
    return (s_send_cb(frame, pay + 4) == VXI11_PORT_RETURN_DEFAULT) ? VXI11_RET_OK : VXI11_RET_ERR;
}

/* --- device_read --- */
static uint32_t handle_device_read(const uint8_t *payload, size_t payload_len, uint32_t xid)
{
    if(payload_len < 64)
    {
        s_ctx.LastErrorCode = VXI11_ERRCODE_BAD_CALL;
        return VXI11_RET_ERR;
    }

    uint32_t request_size = be32_rd(payload + 44);
    if(request_size == 0)
        request_size = VXI11_SCPI_OUT_BUFFER_SIZE;

    uint32_t avail  = (uint32_t)s_scpi_out_len;
    uint32_t send   = (avail <= request_size) ? avail : request_size;
    uint32_t reason = (send < avail) ? VXI11_READ_REASON_PARTIAL : VXI11_READ_REASON_END;
    uint32_t pad    = (4 - (send & 3U)) & 3U;

    uint8_t  frame[32 + 12 + VXI11_SCPI_OUT_BUFFER_SIZE + 4];
    uint32_t hdr = build_reply_common(frame, xid);
    be32_wr(frame + 32, VXI11_ERRCODE_NONE);
    be32_wr(frame + 36, reason);
    be32_wr(frame + 40, send);
    if(send > 0)
        memcpy(frame + 44, s_scpi_out, send);
    if(pad)
        memset(frame + 44 + send, 0, pad);

    size_t pay = hdr + 12 + send + pad; /* 32 + 12 + data + pad */
    finalize_record_marker(frame, pay);

    if(s_send_cb(frame, pay + 4) != VXI11_PORT_RETURN_DEFAULT)
        return VXI11_RET_ERR;

    /* 消费已发送的数据 */
    if(send > 0)
    {
        memmove(s_scpi_out, s_scpi_out + send, s_scpi_out_len - send);
        s_scpi_out_len -= send;
        s_ctx.ScpiOutBytes = (uint32_t)s_scpi_out_len;
    }
    return VXI11_RET_OK;
}

/* Exported functions --------------------------------------------------------*/
const _VXI11_CTX *vxi11_get_ctx(void)
{
    return &s_ctx;
}

void vxi11_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_rx_len       = 0;
    s_scpi_in_len  = 0;
    s_scpi_out_len = 0;

    /* 绑定默认虚拟端口 */
    s_send_cb = vport_send;
    s_getc_cb = vport_getc;

    ring_clear(&s_vrx);
    ring_clear(&s_vtx);
}

void vxi11_bind_port(VXI11_SEND_PTR send_cb, VXI11_GTEC_PTR getc_cb)
{
    s_send_cb = send_cb ? send_cb : vport_send;
    s_getc_cb = getc_cb ? getc_cb : vport_getc;
}

void vxi11_task(void)
{
    /* 1) 从port尽可能拉取字节到内部RPC-RX缓冲 */
    pull_from_port_into_rxbuf( );

    /* 2) 尝试解析尽可能多的完整Record */
    while(1)
    {
        size_t rec_len;
        if(rx_peek_record(&rec_len) != VXI11_RET_OK)
            break;
        process_call(s_rx_buf, rec_len);
        rx_consume(rec_len);
    }
}

/* 取出SCPI输入供外部解析 */
uint32_t vxi11_pop_scpi_input(uint8_t *out, size_t max_len, size_t *actual_len)
{
    if(!out || !actual_len)
        return VXI11_RET_ERR;
    size_t n = (s_scpi_in_len < max_len) ? s_scpi_in_len : max_len;
    if(n > 0)
    {
        memcpy(out, s_scpi_in, n);
        if(n < s_scpi_in_len)
            memmove(s_scpi_in, s_scpi_in + n, s_scpi_in_len - n);
        s_scpi_in_len -= n;
        s_ctx.ScpiInBytes = (uint32_t)s_scpi_in_len;
    }
    *actual_len = n;
    return VXI11_RET_OK;
}

size_t vxi11_peek_scpi_input_len(void)
{
    return s_scpi_in_len;
}

uint32_t vxi11_set_scpi_response(const uint8_t *data, size_t len)
{
    if(!data || len == 0)
    {
        s_scpi_out_len     = 0;
        s_ctx.ScpiOutBytes = 0;
        return VXI11_RET_OK;
    }
    if(len > VXI11_SCPI_OUT_BUFFER_SIZE)
    {
        len                 = VXI11_SCPI_OUT_BUFFER_SIZE;
        s_ctx.LastErrorCode = VXI11_ERRCODE_OVERFLOW;
    }
    memcpy(s_scpi_out, data, len);
    s_scpi_out_len     = len;
    s_ctx.ScpiOutBytes = (uint32_t)s_scpi_out_len;
    return VXI11_RET_OK;
}

size_t vxi11_scpi_response_size(void)
{
    return s_scpi_out_len;
}

void vxi11_clear_scpi_input(void)
{
    s_scpi_in_len     = 0;
    s_ctx.ScpiInBytes = 0;
}
void vxi11_clear_scpi_response(void)
{
    s_scpi_out_len     = 0;
    s_ctx.ScpiOutBytes = 0;
}

/* -------------------- 虚拟端口辅助API -------------------- */
uint32_t vxi11_vport_inject_rx(const uint8_t *data, size_t len)
{
    if(!data || len == 0)
        return VXI11_RET_OK;
    return (ring_push(&s_vrx, data, len) == VXI11_RET_OK) ? VXI11_RET_OK : VXI11_RET_ERR;
}
uint32_t vxi11_vport_drain_tx(uint8_t *out, size_t max_len, size_t *actual_len)
{
    if(!out || !actual_len)
        return VXI11_RET_ERR;
    size_t  want = max_len;
    size_t  got  = 0;
    uint8_t ch;
    while(want && ring_pop_byte(&s_vtx, &ch) == VXI11_RET_OK)
    {
        out[got++] = ch;
        want--;
    }
    *actual_len = got;
    return VXI11_RET_OK;
}
size_t vxi11_vport_peek_tx_len(void)
{
    return ring_peek_len(&s_vtx);
}
size_t vxi11_vport_peek_rx_len(void)
{
    return ring_peek_len(&s_vrx);
}
