#ifndef __LEARNING_GEM5_PIC_PIC_PROTOCOL_HH__
#define __LEARNING_GEM5_PIC_PIC_PROTOCOL_HH__

#include <cstdint>
#include <string>

namespace gem5
{
namespace pic
{

constexpr uint64_t MmioBase = 0x10028000ULL;
constexpr uint64_t MmioWindowSize = 0x100ULL;
constexpr unsigned MmioAccessSize = sizeof(uint64_t);
constexpr uint64_t RetryResponse = UINT64_MAX;

// internal address
constexpr uint64_t InternalBase       = 0x20000000ULL;
constexpr uint64_t InternalWindowSize = 0x1000ULL;
constexpr uint64_t AccModuleAddr           = InternalBase + 0 * InternalWindowSize;  // 0x20000000
constexpr uint64_t CmdStateHelperAddr      = InternalBase + 1 * InternalWindowSize;  // 0x20001000
constexpr uint64_t P2SLModuleAddr          = InternalBase + 2 * InternalWindowSize;  // 0x20002000
constexpr uint64_t P2SRModuleAddr          = InternalBase + 3 * InternalWindowSize;  // 0x20003000
constexpr uint64_t P2SRTModuleAddr         = InternalBase + 4 * InternalWindowSize;  // 0x20004000
constexpr uint64_t CacheControllerAddr     = InternalBase + 5 * InternalWindowSize;  // 0x20005000
constexpr uint64_t CacheBankAddr           = InternalBase + 6 * InternalWindowSize;  // 0x20006000
constexpr uint64_t SwitchControllerAddr    = InternalBase + 7 * InternalWindowSize;  // 0x20007000


enum class SetRegister : uint64_t
{
    Src = 0x00,
    Dst = 0x08,
    Size = 0x10,
    Param = 0x18
};

enum class ModuleID : uint8_t{
    LOAD,
    STORE,
    P2S_L,
    P2S_R,
    P2S_R_T,
    CAL,
    ACC,
    SWITCH,
    QUERY
};

constexpr uint64_t
registerAddress(SetRegister reg)
{
    return MmioBase + static_cast<uint64_t>(reg);
}

constexpr uint64_t
mask(unsigned width)
{
    return width == 64 ? UINT64_MAX : ((UINT64_C(1) << width) - 1);
}

struct SizeFields
{
    uint16_t row;
    uint16_t bytesPerRow;
    uint16_t rowOffset;
};

/* SET_SIZE: row[10:0], bytePerRow[21:11], offset[36:22]. */
constexpr bool
validSize(SizeFields value)
{
    return value.row <= mask(11) && value.bytesPerRow <= mask(11) &&
           value.rowOffset <= mask(15);
}

constexpr uint64_t
packSize(SizeFields value)
{
    return (static_cast<uint64_t>(value.rowOffset & mask(15)) << 22) |
           (static_cast<uint64_t>(value.bytesPerRow & mask(11)) << 11) |
           static_cast<uint64_t>(value.row & mask(11));
}

constexpr SizeFields
unpackSize(uint64_t value)
{
    return {
        static_cast<uint16_t>(value & mask(11)),
        static_cast<uint16_t>((value >> 11) & mask(11)),
        static_cast<uint16_t>((value >> 22) & mask(15))
    };
}

struct ParamFields
{
    ModuleID module;
    uint8_t commandId;
    uint64_t others;
};

/* SET_PARAM: moduleID[3:0], cmdID[11:4], others[63:12]. */
constexpr bool
validParam(ParamFields value)
{
    return static_cast<uint8_t>(value.module) <= mask(4) &&
           value.others <= mask(52);
}

constexpr uint64_t
packParam(ParamFields value)
{
    return (static_cast<uint64_t>(value.module) << 60) |
           ((value.others & mask(52)) << 8) |
           (static_cast<uint64_t>(value.commandId) & mask(8));
}

constexpr ParamFields
unpackParam(uint64_t value)
{
    return {
        static_cast<ModuleID>(value & mask(4)),
        static_cast<uint8_t>((value >> 4) & mask(8)),
        (value >> 12) & mask(52)
    };
}

/* Function-specific contents of SET_PARAM. Names match the generated ISA.c. */
constexpr uint64_t packP2SLParams(uint8_t bitWidthMinusOne)
{
    return bitWidthMinusOne & mask(3);
}

constexpr uint64_t
packP2SRParams(uint8_t buffersPerMat, uint8_t bitWidthMinusOne,
               bool transpose)
{
    return (static_cast<uint64_t>(transpose) << 5) |
           (static_cast<uint64_t>(bitWidthMinusOne & mask(3)) << 2) |
           (buffersPerMat & mask(2));
}

constexpr uint64_t
packIm2ColParams(uint16_t featureSize, uint8_t kernelSize,
                 uint8_t strideSize, uint8_t padding, bool writeToDram)
{
    return (static_cast<uint64_t>(writeToDram) << 21) |
           (static_cast<uint64_t>(padding & mask(4)) << 17) |
           (static_cast<uint64_t>(strideSize & mask(4)) << 13) |
           (static_cast<uint64_t>(kernelSize & mask(4)) << 9) |
           (featureSize & mask(9));
}

constexpr uint64_t
packAccParams(uint8_t sourceCount, uint16_t rowCount,
              uint8_t bitWidthCode)
{
    return (static_cast<uint64_t>(bitWidthCode & mask(3)) << 15) |
           (static_cast<uint64_t>(rowCount & mask(11)) << 4) |
           (sourceCount & mask(4));
}

struct ExeParams
{
    uint16_t validRRows;
    uint8_t buffersPerMat;
    uint8_t calculationsPerMat;
    uint8_t firstRBit;
    uint8_t leftPrecisionMinusOne;
    uint8_t leftBlockRows;
    bool signedLeft;
    bool signedRightLastBit;
    bool accumulator32Bit;
    bool needsNewLeft;
};

constexpr uint64_t
packExeParams(ExeParams value)
{
    return (static_cast<uint64_t>(value.needsNewLeft) << 31) |
           (static_cast<uint64_t>(value.accumulator32Bit) << 30) |
           (static_cast<uint64_t>(value.signedRightLastBit) << 29) |
           (static_cast<uint64_t>(value.signedLeft) << 28) |
           (static_cast<uint64_t>(value.leftBlockRows) << 20) |
           (static_cast<uint64_t>(value.leftPrecisionMinusOne & mask(3)) << 17) |
           (static_cast<uint64_t>(value.firstRBit & mask(3)) << 14) |
           (static_cast<uint64_t>(value.calculationsPerMat & mask(2)) << 12) |
           (static_cast<uint64_t>(value.buffersPerMat & mask(2)) << 10) |
           (value.validRRows & mask(10));
}

constexpr uint64_t packSwitchParams(bool allocate, uint8_t levels)
{
    return (static_cast<uint64_t>(levels & mask(5)) << 1) |
           static_cast<uint64_t>(allocate);
}

constexpr uint64_t packQueryParams(bool immediate)
{
    return static_cast<uint64_t>(immediate);
}

struct QueryResponse
{
    bool commandFinished;
    bool schedulerBusy;
    bool switchSucceeded;
    uint8_t beginPicMatId;
    uint8_t endPicMatId;
};

constexpr QueryResponse
unpackQueryResponse(uint64_t value)
{
    return {
        static_cast<bool>(value & (UINT64_C(1) << 0)),
        static_cast<bool>(value & (UINT64_C(1) << 1)),
        static_cast<bool>(value & (UINT64_C(1) << 2)),
        static_cast<uint8_t>((value >> 3) & mask(6)),
        static_cast<uint8_t>((value >> 9) & mask(6))
    };
}

constexpr uint64_t
packQueryResponse(QueryResponse value)
{
    return static_cast<uint64_t>(value.commandFinished) |
           (static_cast<uint64_t>(value.schedulerBusy) << 1) |
           (static_cast<uint64_t>(value.switchSucceeded) << 2) |
           (static_cast<uint64_t>(value.beginPicMatId & mask(6)) << 3) |
           (static_cast<uint64_t>(value.endPicMatId & mask(6)) << 9);
}

inline const char *moduleName(ModuleID module)
{
    switch (module) {
      case ModuleID::LOAD: return "LOAD";
      case ModuleID::STORE: return "STORE";
      case ModuleID::P2S_L: return "P2S_L";
      case ModuleID::P2S_R: return "P2S_R";
      case ModuleID::P2S_R_T: return "P2S_R_T";;
      case ModuleID::CAL: return "CAL";
      case ModuleID::ACC: return "ACC";
      case ModuleID::SWITCH: return "SWITCH";
      case ModuleID::QUERY: return "QUERY";
      default: return "UNKNOWN";
    }
}

//One PIC transaction accepted by every frontend/transport
struct PicSetRequest
{
    SetRegister reg;
    uint64_t value;
    std::string label;
};

//Raw response to one accepted SET transaction
struct PicSetResponse
{
    uint64_t data;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_PIC_PROTOCOL_HH__
