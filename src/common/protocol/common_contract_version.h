#pragma once

#include <stdint.h>

// common 合约版本只描述两端 common 头文件的布局和语义是否一致。
// 改动协议包、故障位、共享状态语义或单位换算时必须同步主从并提升该版本。
static constexpr uint16_t COMMON_CONTRACT_VERSION = 1;

