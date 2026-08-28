#ifndef __LEARNING_GEM5_PIC_PIC_FUNCTION_BUILDER_HH__
#define __LEARNING_GEM5_PIC_PIC_FUNCTION_BUILDER_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "learning_gem5/PIC/pic_protocol.hh"

namespace gem5
{
namespace pic
{


class PicFunctionBuilder
{
  private:
    static void add(std::vector<PicSetRequest> &out, SetRegister reg,
                    uint64_t value, const std::string &label)
    {
        out.push_back({reg, value, label});
    }

    static void launch(std::vector<PicSetRequest> &out, ModuleId module,
                       uint8_t commandId, uint64_t others)
    {
        add(out, SetRegister::Param,
            packParam({module, commandId, others}),
            std::string("SET_PARAM -> ") + moduleName(module));
    }

  public:
    static std::vector<PicSetRequest>
    load(uint64_t dramSource, uint64_t onChipDestination, SizeFields size,
         uint8_t commandId)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, dramSource, "LOAD SET_SRC");
        add(out, SetRegister::Dst, onChipDestination, "LOAD SET_DST");
        add(out, SetRegister::Size, packSize(size), "LOAD SET_SIZE");
        launch(out, ModuleId::Load, commandId, 0);
        return out;
    }

    static std::vector<PicSetRequest>
    p2sl(uint64_t dramSource, uint64_t onChipDestination, uint16_t rows,
         uint16_t rowOffset, uint8_t commandId, uint8_t bitWidthMinusOne)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, dramSource, "P2SL SET_SRC");
        add(out, SetRegister::Dst, onChipDestination, "P2SL SET_DST");
        add(out, SetRegister::Size, packSize({rows, rowOffset, 0}),
            "P2SL SET_SIZE");
        launch(out, ModuleId::P2SL, commandId,
               packP2SLParams(bitWidthMinusOne));
        return out;
    }

    static std::vector<PicSetRequest>
    p2sr(uint64_t dramSource, uint64_t onChipDestination, SizeFields size,
         uint8_t commandId, uint8_t buffersPerMat,
         uint8_t bitWidthMinusOne, bool transpose)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, dramSource, "P2SR SET_SRC");
        add(out, SetRegister::Dst, onChipDestination, "P2SR SET_DST");
        add(out, SetRegister::Size, packSize(size), "P2SR SET_SIZE");
        launch(out, ModuleId::P2SR, commandId,
               packP2SRParams(buffersPerMat, bitWidthMinusOne, transpose));
        return out;
    }

    static std::vector<PicSetRequest>
    im2col(uint64_t dramSource, uint64_t dramDestination,
           uint16_t featureSize, uint8_t kernelSize, uint8_t strideSize,
           uint8_t padding, bool writeToDram)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, dramSource, "IM2COL SET_SRC");
        add(out, SetRegister::Dst, dramDestination, "IM2COL SET_DST");
        launch(out, ModuleId::Im2Col, 0,
               packIm2ColParams(featureSize, kernelSize, strideSize,
                                padding, writeToDram));
        return out;
    }

    static std::vector<PicSetRequest>
    acc(uint64_t onChipSource, uint64_t onChipDestination,
        uint8_t commandId, uint8_t sourceCount, uint16_t rowCount,
        uint8_t bitWidthCode)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, onChipSource, "ACC SET_SRC");
        add(out, SetRegister::Dst, onChipDestination, "ACC SET_DST");
        launch(out, ModuleId::Acc, commandId,
               packAccParams(sourceCount, rowCount, bitWidthCode));
        return out;
    }

    static std::vector<PicSetRequest>
    exe(uint64_t leftBase, uint8_t targetMatId, uint8_t commandId,
        ExeParams params)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, leftBase, "EXE SET_SRC");
        add(out, SetRegister::Dst, targetMatId, "EXE SET_DST");
        launch(out, ModuleId::Exe, commandId, packExeParams(params));
        return out;
    }

    static std::vector<PicSetRequest>
    store(uint64_t onChipSource, uint64_t dramDestination, SizeFields size,
          uint8_t commandId)
    {
        std::vector<PicSetRequest> out;
        add(out, SetRegister::Src, onChipSource, "STORE SET_SRC");
        add(out, SetRegister::Dst, dramDestination, "STORE SET_DST");
        add(out, SetRegister::Size, packSize(size), "STORE SET_SIZE");
        launch(out, ModuleId::Store, commandId, 0);
        return out;
    }

    static std::vector<PicSetRequest> switchMode(bool allocate, uint8_t levels)
    {
        std::vector<PicSetRequest> out;
        launch(out, ModuleId::Switch, 0, packSwitchParams(allocate, levels));
        return out;
    }

    static std::vector<PicSetRequest> query(uint8_t commandId, bool immediate)
    {
        std::vector<PicSetRequest> out;
        launch(out, ModuleId::Query, commandId, packQueryParams(immediate));
        return out;
    }
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_PIC_FUNCTION_BUILDER_HH__
