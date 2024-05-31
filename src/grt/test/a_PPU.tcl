# check route guides for gcd_nangate45. def file from the openroad-flow
#source "helpers.tcl"
#read_lef "Nangate45/Nangate45.lef"


read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_fd_sc_hs.tlef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__com_bus_slice_10um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__corner_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vccd_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssa_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssio_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_sram_1rw1r_80x64_8.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__com_bus_slice_1um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__disconnect_vccd_slice_5um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vdda_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssa_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_fd_io__top_xres4v2.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130io_fill.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__com_bus_slice_20um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__disconnect_vdda_slice_5um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vdda_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssd_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_fd_sc_hd_merged.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_sram_1rw1r_128x256_8.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__com_bus_slice_5um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__gpiov2_pad_wrapped.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vddio_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssd_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_sram_1rw1r_44x64_8.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__connect_vcchib_vccd_and_vswitch_vddio_slice_20um.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vccd_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vddio_lvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_ef_io__vssio_hvc_pad.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_fd_sc_hs_merged.lef"
read_lef "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lef/sky130_sram_1rw1r_64x256_8.lef"

 
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_dummy_io.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_fd_sc_hs__tt_025C_1v80.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_sram_1rw1r_128x256_8_TT_1p8V_25C.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_sram_1rw1r_64x256_8_TT_1p8V_25C.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_fd_sc_hd__tt_025C_1v80.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_fd_sc_hs__tt_100C_1v80.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_sram_1rw1r_44x64_8_TT_1p8V_25C.lib",
read_liberty "/home/huangzengrong/a08_3rd_test/irefactor/scripts/sky130/lib/sky130_sram_1rw1r_80x64_8_TT_1p8V_25C.lib"

read_sdc /home/wuhongxi/OpenROAD/sky130/sdc/PPU.sdc

read_def /home/wuhongxi/OpenROAD/sky130/def/PPU.def

set guide_file [make_result_file gcd_flute.guide]

set_routing_alpha 1.0
global_route -verbose
estimate_parasitics -global_routing

write_guides $guide_file


report_wns
report_tns
