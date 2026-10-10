#pragma once
#include <cstdint>

namespace esphome {
namespace sy7t609 {

// ============================================================================
// 校准 / 缩放寄存器固定值
// reset_calibration 动作会把这些值写回芯片并存入 flash。
// 如需修改校准，直接改这里的常量即可（更复杂的运行时校准放在工程版本中实现）。
// ============================================================================
constexpr uint32_t DEFAULT_ISCALE       = 0x00F230;
constexpr uint32_t DEFAULT_VSCALE       = 0x0A2D78;
constexpr uint32_t DEFAULT_PSCALE       = 0x027703;
constexpr uint32_t DEFAULT_ACCUM        = 0x001A2C;
constexpr uint32_t DEFAULT_IRMS_TARGET  = 0x0003E8;
constexpr uint32_t DEFAULT_VRMS_TARGET  = 0x03807C;
constexpr uint32_t DEFAULT_POWER_TARGET = 0x01D4C0;
constexpr uint32_t DEFAULT_CONTROL      = 0x001817;
constexpr uint32_t DEFAULT_BUCKETH      = 0x000247;
constexpr uint32_t DEFAULT_BUCKETL      = 0x7780A5;  // 原厂默认数据
constexpr uint32_t DEFAULT_IGAIN        = 0x43DF0D;  // 使用万用表校准的数据
constexpr uint32_t DEFAULT_VGAIN        = 0x210A40;

// ============================================================================
// SSI 协议常量
// ============================================================================
constexpr uint8_t SSI_HEADER               = 0xAA;
constexpr uint8_t SSI_DEFAULT_FRAME_SIZE   = 3;
constexpr uint8_t SSI_MAX_PAYLOAD_SIZE     = 7;
constexpr uint8_t SSI_READ_PAYLOAD_SIZE    = 4;
constexpr uint8_t SSI_REPLY_PAYLOAD_SIZE   = 3;
constexpr uint8_t SSI_WRITE_PAYLOAD_SIZE   = 7;
constexpr uint8_t SSI_UART_READ_SEND_PKG_SIZE  = SSI_DEFAULT_FRAME_SIZE + SSI_READ_PAYLOAD_SIZE;
constexpr uint8_t SSI_UART_WRITE_SEND_PKG_SIZE = SSI_DEFAULT_FRAME_SIZE + SSI_WRITE_PAYLOAD_SIZE;
constexpr uint8_t SSI_UART_READ_RECV_PKG_SIZE  = SSI_DEFAULT_FRAME_SIZE + SSI_REPLY_PAYLOAD_SIZE;

// ============================================================================
// SSI 命令字
// ============================================================================
constexpr uint8_t CMD_SELECT_REGISTER_ADDRESS = 0xA3;
constexpr uint8_t CMD_READ_REGITSTER_3BYTES   = 0xE3;
constexpr uint8_t CMD_WRITE_RETISTER_3BYTES   = 0xD3;

// ============================================================================
// 寄存器地址表
// ============================================================================
enum sy7t609_register_map {
    ADDR_COMMAND       = 0x0000,
    ADDR_CONTROL       = 0x0006,
    // Metering Address
    ADDR_CTEMP         = 0x0027,
    ADDR_VAVG          = 0x002D,
    ADDR_IAVG          = 0x0030,
    ADDR_VRMS          = 0x0033,
    ADDR_IRMS          = 0x0036,
    ADDR_POWER         = 0x0039,
    ADDR_VAR           = 0x003C,
    ADDR_VA            = 0x003F,  // Scaled Apparent Power（视在功率，S24，÷1000 得 VA）
    ADDR_FREQUENCY     = 0x0042,  // Scaled Line Frequency（线路频率，U24，÷1000 得 Hz）
    ADDR_PF            = 0x0048,
    ADDR_EPPCNT        = 0x0069,  // Positive Active Energy Count（正向有功电量，用电）
    ADDR_EPMCNT        = 0x006C,  // Negative Active Energy Count（反向有功电量，倒送）
    // Calibration Address
    ADDR_BUCKETL       = 0x00C0,
    ADDR_BUCKETH       = 0x00C3,
    ADDR_IGAIN         = 0x00D5,
    ADDR_VGAIN         = 0x00D8,
    ADDR_ISCALE        = 0x00ED,
    ADDR_VSCALE        = 0x00F0,
    ADDR_PSCALE        = 0x00F3,
    ADDR_ACCUM         = 0x0105,
    ADDR_IAVG_TARGET   = 0x0111,
    ADDR_VAVG_TARGET   = 0x0114,
    ADDR_IRMS_TARGET   = 0x0117,
    ADDR_VRMS_TARGET   = 0x011A,
    ADDR_POWER_TARGET  = 0x011D,
    ADDR_ERROR         = 0x0FFF
};

// ============================================================================
// COMMAND 寄存器命令码
// ============================================================================
enum command_register_code {
    CMD_NONE                       = 0x000000,
    CMD_REG_CLEAR_ENGERGY_COUNTERS = 0xEC0000,  // 清除所有电能计数器
    CMD_REG_SOFT_RESET             = 0xBD0000,  // 软复位
    CMD_REG_SAVE_TO_FLASH          = 0xACC200   // 保存当前寄存器到 flash
};

// ============================================================================
// SSI 回复码
// ============================================================================
enum sy7t609_reply_code {
    REPLY_ACK_WITH_DATA           = 0xAA,
    REPLY_AUTO_REPORTING_HEADER   = 0xAE,
    REPLY_ACK_WITHOUT_DATA        = 0xAD,
    REPLY_NEGATIVE_ACK            = 0xB0,
    REPLY_COMMAND_NOT_IMPLEMENTED = 0xBC,
    REPLY_CHECKSUM_FAILED         = 0xBD,
    REPLY_BUFFER_OVERFLOW         = 0xBF
};

// ============================================================================
// 内部读取状态机（仅用于读取测量值的轮询；写操作为同步执行，不占用此状态）
// ============================================================================
enum class ProcessState : uint8_t {
    IDLE = 0,   // 空闲，可发起新一轮读取
    READING,    // 正在等待某个寄存器的读取回复
};

} //sy7t609
} //esphome
