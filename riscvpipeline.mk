#=========================================================================
# riscvpipeline Subpackage
#=========================================================================

riscvpipeline_deps = \
  vc \

riscvpipeline_srcs = \
  riscvpipeline-CoreDpath.v \
  riscvpipeline-CoreDpathRegfile.v \
  riscvpipeline-CoreDpathAlu.v \
  riscvpipeline-CoreDpathMulDiv.v \
  riscvpipeline-CoreCtrl.v \
  riscvpipeline-Core.v \
  riscvpipeline-InstMsg.v \

riscvpipeline_test_srcs = \
  riscvpipeline-InstMsg.t.v \

riscvpipeline_prog_srcs = \
  riscvpipeline-sim.v