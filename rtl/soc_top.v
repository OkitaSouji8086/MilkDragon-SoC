/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

`include "config.h"

module soc_top #(
    parameter SIMULATION = 1'b0
) (
    input clk,   //50MHz 时钟输入
    input reset, //BTN6手动复位按钮开关，带消抖电路，按下时为1

    //图像输出信号
    output [2:0] video_red,    //红色像素，3位
    output [2:0] video_green,  //绿色像素，3位
    output [1:0] video_blue,   //蓝色像素，2位
    output       video_hsync,  //行同步（水平同步）信号
    output       video_vsync,  //场同步（垂直同步）信号
    output       video_clk,    //像素时钟输出
    output       video_de,     //行数据有效信号，用于区分消隐区

    input         clock_btn,  //BTN5手动时钟按钮开关，带消抖电路，按下时为1
    input  [ 3:0] touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
    input  [31:0] dip_sw,     //32位拨码开关，拨到“ON”时为1
    output [15:0] leds,       //16位LED，输出时1点亮
    output [ 7:0] dpy0,       //数码管低位信号，包括小数点，输出1点亮
    output [ 7:0] dpy1,       //数码管高位信号，包括小数点，输出1点亮

    //BaseRAM信号
    inout [31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output [19:0] base_ram_addr,  //BaseRAM地址
    output [ 3:0]   base_ram_be_n,      //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output base_ram_ce_n,  //BaseRAM片选，低有效
    output base_ram_oe_n,  //BaseRAM读使能，低有效
    output base_ram_we_n,  //BaseRAM写使能，低有效
    //ExtRAM信号
    inout [31:0] ext_ram_data,  //ExtRAM数据
    output [19:0] ext_ram_addr,  //ExtRAM地址
    output [ 3:0]   ext_ram_be_n,       //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output ext_ram_ce_n,  //ExtRAM片选，低有效
    output ext_ram_oe_n,  //ExtRAM读使能，低有效
    output ext_ram_we_n,  //ExtRAM写使能，低有效

    //Flash存储器信号，参考 JS28F640 芯片手册
    output [22:0] flash_a,  //Flash地址，a0仅在8bit模式有效，16bit模式无意义
    inout [15:0] flash_d,  //Flash数据
    output flash_rp_n,  //Flash复位信号，低有效
    output flash_vpen,  //Flash写保护信号，低电平时不能擦除、烧写
    output flash_ce_n,  //Flash片选信号，低有效
    output flash_oe_n,  //Flash读使能信号，低有效
    output flash_we_n,  //Flash写使能信号，低有效
    output          flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash的16位模式时请设为1

    //------uart-------
    inout UART_RX,  //串口RX接收
    inout UART_TX   //串口TX发送
);


  //================================================================================
  // INTERNAL SIGNALS
  //================================================================================
  //--------------------------------------------------------------------------------
  // CLK & RST
  //--------------------------------------------------------------------------------
  wire         cpu_clk;
  wire         cpu_resetn;
  wire         sys_clk;
  wire         sys_resetn;
  //--------------------------------------------------------------------------------
  // BUS MATRIX
  //--------------------------------------------------------------------------------
  wire         axi_mtx_mst0_awvalid;
  wire         axi_mtx_mst0_awready;
  wire [ 31:0] axi_mtx_mst0_awaddr;
  wire [  3:0] axi_mtx_mst0_awid;
  wire [  7:0] axi_mtx_mst0_awlen;
  wire [  2:0] axi_mtx_mst0_awsize;
  wire [  1:0] axi_mtx_mst0_awburst;
  wire [  0:0] axi_mtx_mst0_awlock;
  wire [  3:0] axi_mtx_mst0_awcache;
  wire [  2:0] axi_mtx_mst0_awprot;
  wire [  3:0] axi_mtx_mst0_awqos;
  wire         axi_mtx_mst0_wvalid;
  wire         axi_mtx_mst0_wready;
  wire [ 31:0] axi_mtx_mst0_wdata;
  wire [  3:0] axi_mtx_mst0_wstrb;
  wire         axi_mtx_mst0_wlast;
  wire         axi_mtx_mst0_bvalid;
  wire         axi_mtx_mst0_bready;
  wire [  3:0] axi_mtx_mst0_bid;
  wire [  1:0] axi_mtx_mst0_bresp;
  wire         axi_mtx_mst0_arvalid;
  wire         axi_mtx_mst0_arready;
  wire [ 31:0] axi_mtx_mst0_araddr;
  wire [  3:0] axi_mtx_mst0_arid;
  wire [  7:0] axi_mtx_mst0_arlen;
  wire [  2:0] axi_mtx_mst0_arsize;
  wire [  1:0] axi_mtx_mst0_arburst;
  wire [  0:0] axi_mtx_mst0_arlock;
  wire [  3:0] axi_mtx_mst0_arcache;
  wire [  2:0] axi_mtx_mst0_arprot;
  wire [  3:0] axi_mtx_mst0_arqos;
  wire         axi_mtx_mst0_rvalid;
  wire         axi_mtx_mst0_rready;
  wire [ 31:0] axi_mtx_mst0_rdata;
  wire [  3:0] axi_mtx_mst0_rid;
  wire [  1:0] axi_mtx_mst0_rresp;
  wire         axi_mtx_mst0_rlast;
  wire         axi_mtx_mst1_awvalid;
  wire         axi_mtx_mst1_awready;
  wire [ 31:0] axi_mtx_mst1_awaddr;
  wire [  3:0] axi_mtx_mst1_awid;
  wire [  7:0] axi_mtx_mst1_awlen;
  wire [  2:0] axi_mtx_mst1_awsize;
  wire [  1:0] axi_mtx_mst1_awburst;
  wire [  0:0] axi_mtx_mst1_awlock;
  wire [  3:0] axi_mtx_mst1_awcache;
  wire [  2:0] axi_mtx_mst1_awprot;
  wire [  3:0] axi_mtx_mst1_awqos;
  wire         axi_mtx_mst1_wvalid;
  wire         axi_mtx_mst1_wready;
  wire [ 31:0] axi_mtx_mst1_wdata;
  wire [  3:0] axi_mtx_mst1_wstrb;
  wire         axi_mtx_mst1_wlast;
  wire         axi_mtx_mst1_bvalid;
  wire         axi_mtx_mst1_bready;
  wire [  3:0] axi_mtx_mst1_bid;
  wire [  1:0] axi_mtx_mst1_bresp;
  wire         axi_mtx_mst1_arvalid;
  wire         axi_mtx_mst1_arready;
  wire [ 31:0] axi_mtx_mst1_araddr;
  wire [  3:0] axi_mtx_mst1_arid;
  wire [  7:0] axi_mtx_mst1_arlen;
  wire [  2:0] axi_mtx_mst1_arsize;
  wire [  1:0] axi_mtx_mst1_arburst;
  wire [  0:0] axi_mtx_mst1_arlock;
  wire [  3:0] axi_mtx_mst1_arcache;
  wire [  2:0] axi_mtx_mst1_arprot;
  wire [  3:0] axi_mtx_mst1_arqos;
  wire         axi_mtx_mst1_rvalid;
  wire         axi_mtx_mst1_rready;
  wire [ 31:0] axi_mtx_mst1_rdata;
  wire [  3:0] axi_mtx_mst1_rid;
  wire [  1:0] axi_mtx_mst1_rresp;
  wire         axi_mtx_mst1_rlast;
  wire         axi_mtx_slv0_awvalid;
  wire         axi_mtx_slv0_awready;
  wire [ 31:0] axi_mtx_slv0_awaddr;
  wire [  4:0] axi_mtx_slv0_awid;
  wire [  7:0] axi_mtx_slv0_awlen;
  wire [  2:0] axi_mtx_slv0_awsize;
  wire [  1:0] axi_mtx_slv0_awburst;
  wire [  0:0] axi_mtx_slv0_awlock;
  wire [  3:0] axi_mtx_slv0_awcache;
  wire [  2:0] axi_mtx_slv0_awprot;
  wire [  3:0] axi_mtx_slv0_awregion;
  wire [  3:0] axi_mtx_slv0_awqos;
  wire         axi_mtx_slv0_wvalid;
  wire         axi_mtx_slv0_wready;
  wire [ 31:0] axi_mtx_slv0_wdata;
  wire [  3:0] axi_mtx_slv0_wstrb;
  wire         axi_mtx_slv0_wlast;
  wire         axi_mtx_slv0_bvalid;
  wire         axi_mtx_slv0_bready;
  wire [  4:0] axi_mtx_slv0_bid;
  wire [  1:0] axi_mtx_slv0_bresp;
  wire         axi_mtx_slv0_arvalid;
  wire         axi_mtx_slv0_arready;
  wire [ 31:0] axi_mtx_slv0_araddr;
  wire [  4:0] axi_mtx_slv0_arid;
  wire [  7:0] axi_mtx_slv0_arlen;
  wire [  2:0] axi_mtx_slv0_arsize;
  wire [  1:0] axi_mtx_slv0_arburst;
  wire [  0:0] axi_mtx_slv0_arlock;
  wire [  3:0] axi_mtx_slv0_arcache;
  wire [  2:0] axi_mtx_slv0_arprot;
  wire [  3:0] axi_mtx_slv0_arregion;
  wire [  3:0] axi_mtx_slv0_arqos;
  wire         axi_mtx_slv0_rvalid;
  wire         axi_mtx_slv0_rready;
  wire [ 31:0] axi_mtx_slv0_rdata;
  wire [  4:0] axi_mtx_slv0_rid;
  wire [  1:0] axi_mtx_slv0_rresp;
  wire         axi_mtx_slv0_rlast;
  wire         axi_mtx_slv1_awvalid;
  wire         axi_mtx_slv1_awready;
  wire [ 31:0] axi_mtx_slv1_awaddr;
  wire [  4:0] axi_mtx_slv1_awid;
  wire [  7:0] axi_mtx_slv1_awlen;
  wire [  2:0] axi_mtx_slv1_awsize;
  wire [  1:0] axi_mtx_slv1_awburst;
  wire [  0:0] axi_mtx_slv1_awlock;
  wire [  3:0] axi_mtx_slv1_awcache;
  wire [  2:0] axi_mtx_slv1_awprot;
  wire [  3:0] axi_mtx_slv1_awregion;
  wire [  3:0] axi_mtx_slv1_awqos;
  wire         axi_mtx_slv1_wvalid;
  wire         axi_mtx_slv1_wready;
  wire [ 31:0] axi_mtx_slv1_wdata;
  wire [  3:0] axi_mtx_slv1_wstrb;
  wire         axi_mtx_slv1_wlast;
  wire         axi_mtx_slv1_bvalid;
  wire         axi_mtx_slv1_bready;
  wire [  4:0] axi_mtx_slv1_bid;
  wire [  1:0] axi_mtx_slv1_bresp;
  wire         axi_mtx_slv1_arvalid;
  wire         axi_mtx_slv1_arready;
  wire [ 31:0] axi_mtx_slv1_araddr;
  wire [  4:0] axi_mtx_slv1_arid;
  wire [  7:0] axi_mtx_slv1_arlen;
  wire [  2:0] axi_mtx_slv1_arsize;
  wire [  1:0] axi_mtx_slv1_arburst;
  wire [  0:0] axi_mtx_slv1_arlock;
  wire [  3:0] axi_mtx_slv1_arcache;
  wire [  2:0] axi_mtx_slv1_arprot;
  wire [  3:0] axi_mtx_slv1_arregion;
  wire [  3:0] axi_mtx_slv1_arqos;
  wire         axi_mtx_slv1_rvalid;
  wire         axi_mtx_slv1_rready;
  wire [ 31:0] axi_mtx_slv1_rdata;
  wire [  4:0] axi_mtx_slv1_rid;
  wire [  1:0] axi_mtx_slv1_rresp;
  wire         axi_mtx_slv1_rlast;
  wire         axi_mtx_slv2_awvalid;
  wire         axi_mtx_slv2_awready;
  wire [ 31:0] axi_mtx_slv2_awaddr;
  wire [  4:0] axi_mtx_slv2_awid;
  wire [  7:0] axi_mtx_slv2_awlen;
  wire [  2:0] axi_mtx_slv2_awsize;
  wire [  1:0] axi_mtx_slv2_awburst;
  wire [  0:0] axi_mtx_slv2_awlock;
  wire [  3:0] axi_mtx_slv2_awcache;
  wire [  2:0] axi_mtx_slv2_awprot;
  wire [  3:0] axi_mtx_slv2_awregion;
  wire [  3:0] axi_mtx_slv2_awqos;
  wire         axi_mtx_slv2_wvalid;
  wire         axi_mtx_slv2_wready;
  wire [ 31:0] axi_mtx_slv2_wdata;
  wire [  3:0] axi_mtx_slv2_wstrb;
  wire         axi_mtx_slv2_wlast;
  wire         axi_mtx_slv2_bvalid;
  wire         axi_mtx_slv2_bready;
  wire [  4:0] axi_mtx_slv2_bid;
  wire [  1:0] axi_mtx_slv2_bresp;
  wire         axi_mtx_slv2_arvalid;
  wire         axi_mtx_slv2_arready;
  wire [ 31:0] axi_mtx_slv2_araddr;
  wire [  4:0] axi_mtx_slv2_arid;
  wire [  7:0] axi_mtx_slv2_arlen;
  wire [  2:0] axi_mtx_slv2_arsize;
  wire [  1:0] axi_mtx_slv2_arburst;
  wire [  0:0] axi_mtx_slv2_arlock;
  wire [  3:0] axi_mtx_slv2_arcache;
  wire [  2:0] axi_mtx_slv2_arprot;
  wire [  3:0] axi_mtx_slv2_arregion;
  wire [  3:0] axi_mtx_slv2_arqos;
  wire         axi_mtx_slv2_rvalid;
  wire         axi_mtx_slv2_rready;
  wire [ 31:0] axi_mtx_slv2_rdata;
  wire [  4:0] axi_mtx_slv2_rid;
  wire [  1:0] axi_mtx_slv2_rresp;
  wire         axi_mtx_slv2_rlast;
  wire         axi_mtx_slv3_awvalid;
  wire         axi_mtx_slv3_awready;
  wire [ 31:0] axi_mtx_slv3_awaddr;
  wire [  4:0] axi_mtx_slv3_awid;
  wire [  7:0] axi_mtx_slv3_awlen;
  wire [  2:0] axi_mtx_slv3_awsize;
  wire [  1:0] axi_mtx_slv3_awburst;
  wire [  0:0] axi_mtx_slv3_awlock;
  wire [  3:0] axi_mtx_slv3_awcache;
  wire [  2:0] axi_mtx_slv3_awprot;
  wire [  3:0] axi_mtx_slv3_awregion;
  wire [  3:0] axi_mtx_slv3_awqos;
  wire         axi_mtx_slv3_wvalid;
  wire         axi_mtx_slv3_wready;
  wire [ 31:0] axi_mtx_slv3_wdata;
  wire [  3:0] axi_mtx_slv3_wstrb;
  wire         axi_mtx_slv3_wlast;
  wire         axi_mtx_slv3_bvalid;
  wire         axi_mtx_slv3_bready;
  wire [  4:0] axi_mtx_slv3_bid;
  wire [  1:0] axi_mtx_slv3_bresp;
  wire         axi_mtx_slv3_arvalid;
  wire         axi_mtx_slv3_arready;
  wire [ 31:0] axi_mtx_slv3_araddr;
  wire [  4:0] axi_mtx_slv3_arid;
  wire [  7:0] axi_mtx_slv3_arlen;
  wire [  2:0] axi_mtx_slv3_arsize;
  wire [  1:0] axi_mtx_slv3_arburst;
  wire [  0:0] axi_mtx_slv3_arlock;
  wire [  3:0] axi_mtx_slv3_arcache;
  wire [  2:0] axi_mtx_slv3_arprot;
  wire [  3:0] axi_mtx_slv3_arregion;
  wire [  3:0] axi_mtx_slv3_arqos;
  wire         axi_mtx_slv3_rvalid;
  wire         axi_mtx_slv3_rready;
  wire [ 31:0] axi_mtx_slv3_rdata;
  wire [  4:0] axi_mtx_slv3_rid;
  wire [  1:0] axi_mtx_slv3_rresp;
  wire         axi_mtx_slv3_rlast;
  wire [  7:0] s_axi_awid;
  wire [ 63:0] s_axi_awaddr;
  wire [ 15:0] s_axi_awlen;
  wire [  5:0] s_axi_awsize;
  wire [  3:0] s_axi_awburst;
  wire [  1:0] s_axi_awlock;
  wire [  7:0] s_axi_awcache;
  wire [  5:0] s_axi_awprot;
  wire [  7:0] s_axi_awqos;
  wire [  1:0] s_axi_awvalid;
  wire [  1:0] s_axi_awready;
  wire [ 63:0] s_axi_wdata;
  wire [  7:0] s_axi_wstrb;
  wire [  1:0] s_axi_wlast;
  wire [  1:0] s_axi_wvalid;
  wire [  1:0] s_axi_wready;
  wire [  7:0] s_axi_bid;
  wire [  3:0] s_axi_bresp;
  wire [  1:0] s_axi_bvalid;
  wire [  1:0] s_axi_bready;
  wire [  7:0] s_axi_arid;
  wire [ 63:0] s_axi_araddr;
  wire [ 15:0] s_axi_arlen;
  wire [  5:0] s_axi_arsize;
  wire [  3:0] s_axi_arburst;
  wire [  1:0] s_axi_arlock;
  wire [  7:0] s_axi_arcache;
  wire [  5:0] s_axi_arprot;
  wire [  7:0] s_axi_arqos;
  wire [  1:0] s_axi_arvalid;
  wire [  1:0] s_axi_arready;
  wire [  7:0] s_axi_rid;
  wire [ 63:0] s_axi_rdata;
  wire [  3:0] s_axi_rresp;
  wire [  1:0] s_axi_rlast;
  wire [  1:0] s_axi_rvalid;
  wire [  1:0] s_axi_rready;
  wire [127:0] m_axi_awaddr;
  wire [ 31:0] m_axi_awlen;
  wire [ 11:0] m_axi_awsize;
  wire [  7:0] m_axi_awburst;
  wire [  3:0] m_axi_awlock;
  wire [ 15:0] m_axi_awcache;
  wire [ 11:0] m_axi_awprot;
  wire [ 15:0] m_axi_awregion;
  wire [ 15:0] m_axi_awqos;
  wire [  3:0] m_axi_awvalid;
  wire [  3:0] m_axi_awready;
  wire [127:0] m_axi_wdata;
  wire [ 15:0] m_axi_wstrb;
  wire [  3:0] m_axi_wlast;
  wire [  3:0] m_axi_wvalid;
  wire [  3:0] m_axi_wready;
  wire [  7:0] m_axi_bresp;
  wire [  3:0] m_axi_bvalid;
  wire [  3:0] m_axi_bready;
  wire [127:0] m_axi_araddr;
  wire [ 31:0] m_axi_arlen;
  wire [ 11:0] m_axi_arsize;
  wire [  7:0] m_axi_arburst;
  wire [  3:0] m_axi_arlock;
  wire [ 15:0] m_axi_arcache;
  wire [ 11:0] m_axi_arprot;
  wire [ 15:0] m_axi_arregion;
  wire [ 15:0] m_axi_arqos;
  wire [  3:0] m_axi_arvalid;
  wire [  3:0] m_axi_arready;
  wire [127:0] m_axi_rdata;
  wire [  7:0] m_axi_rresp;
  wire [  3:0] m_axi_rlast;
  wire [  3:0] m_axi_rvalid;
  wire [  3:0] m_axi_rready;
  //--------------------------------------------------------------------------------
  // AXI to APB
  //--------------------------------------------------------------------------------
  wire [ 31:0] x2p_axi_awaddr;
  wire         x2p_axi_awvalid;
  wire         x2p_axi_awready;
  wire [ 31:0] x2p_axi_wdata;
  wire         x2p_axi_wvalid;
  wire         x2p_axi_wready;
  wire [  1:0] x2p_axi_bresp;
  wire         x2p_axi_bvalid;
  wire         x2p_axi_bready;
  wire [ 31:0] x2p_axi_araddr;
  wire         x2p_axi_arvalid;
  wire         x2p_axi_arready;
  wire [ 31:0] x2p_axi_rdata;
  wire [  1:0] x2p_axi_rresp;
  wire         x2p_axi_rvalid;
  wire         x2p_axi_rready;
  wire [ 31:0] x2p_apb_paddr;
  wire [  3:0] x2p_apb_psel;
  wire         x2p_apb_penable;
  wire         x2p_apb_pwrite;
  wire [ 31:0] x2p_apb_pwdata;
  wire [  3:0] x2p_apb_pready;
  wire [ 31:0] x2p_apb_prdata;
  wire [ 31:0] x2p_apb_prdata2;
  wire [ 31:0] x2p_apb_prdata3;
  wire [ 31:0] x2p_apb_prdata4;
  wire [  3:0] x2p_apb_pslverr;
  //--------------------------------------------------------------------------------
  // CPU
  //--------------------------------------------------------------------------------
  wire         cpu_awvalid;
  wire         cpu_awready;
  wire [ 31:0] cpu_awaddr;
  wire [  3:0] cpu_awid;
  wire [  7:0] cpu_awlen;
  wire [  2:0] cpu_awsize;
  wire [  1:0] cpu_awburst;
  wire [  0:0] cpu_awlock;
  wire [  3:0] cpu_awcache;
  wire [  2:0] cpu_awprot;
  wire         cpu_wvalid;
  wire         cpu_wready;
  wire [ 31:0] cpu_wdata;
  wire [  3:0] cpu_wstrb;
  wire         cpu_wlast;
  wire         cpu_bvalid;
  wire         cpu_bready;
  wire [  3:0] cpu_bid;
  wire [  1:0] cpu_bresp;
  wire         cpu_arvalid;
  wire         cpu_arready;
  wire [ 31:0] cpu_araddr;
  wire [  3:0] cpu_arid;
  wire [  7:0] cpu_arlen;
  wire [  2:0] cpu_arsize;
  wire [  1:0] cpu_arburst;
  wire [  0:0] cpu_arlock;
  wire [  3:0] cpu_arcache;
  wire [  2:0] cpu_arprot;
  wire         cpu_rvalid;
  wire         cpu_rready;
  wire [ 31:0] cpu_rdata;
  wire [  3:0] cpu_rid;
  wire [  1:0] cpu_rresp;
  wire         cpu_rlast;
  wire [  7:0] intrpt;
  wire [ 31:0] debug0_wb_pc;
  wire [  3:0] debug0_wb_rf_wen;
  wire [  4:0] debug0_wb_rf_wnum;
  wire [ 31:0] debug0_wb_rf_wdata;
  wire [ 31:0] debug0_wb_inst;
  //--------------------------------------------------------------------------------
  // DMAC
  //--------------------------------------------------------------------------------
  wire [1-1:0] dmac_int;
  wire [ 31:1] periph_tx_req;
  wire [ 31:1] periph_tx_clr;
  wire [ 31:1] periph_rx_req;
  wire [ 31:1] periph_rx_clr;
  wire         dmac_pclken;
  wire         dmac_psel;
  wire         dmac_penable;
  wire [ 12:0] dmac_paddr;
  wire         dmac_pwrite;
  wire [ 31:0] dmac_pwdata;
  wire [ 31:0] dmac_prdata;
  wire         dmac_pslverr;
  wire         dmac_pready;
  //--------------------------------------------------------------------------------
  // UART
  //--------------------------------------------------------------------------------
  wire UART_CTS, UART_RTS;
  wire UART_DTR, UART_DSR;
  wire UART_RI, UART_DCD;
  wire uart0_int;
  wire uart0_txd_o;
  wire uart0_txd_i;
  wire uart0_txd_oe;
  wire uart0_rxd_o;
  wire uart0_rxd_i;
  wire uart0_rxd_oe;
  wire uart0_rts_o;
  wire uart0_cts_i;
  wire uart0_dsr_i;
  wire uart0_dcd_i;
  wire uart0_dtr_o;
  wire uart0_ri_i;
  //--------------------------------------------------------------------------------
  // CONFREG
  //--------------------------------------------------------------------------------
  wire confreg_int;


  //================================================================================
  // MODULE INSTANTIATIONS
  //================================================================================
  //--------------------------------------------------------------------------------
  // PLL
  //--------------------------------------------------------------------------------
  generate
    if (SIMULATION) begin : sim_clk
      //simulation clk.
      reg clk_sim;
      initial begin
        clk_sim = 1'b0;
      end
      always #15 clk_sim = ~clk_sim;

      assign cpu_clk = clk_sim;
      assign sys_clk = clk;
      rst_sync u_rst_sys (
          .clk(sys_clk),
          .rst_n_in(~reset),
          .rst_n_out(sys_resetn)
      );
      rst_sync u_rst_cpu (
          .clk(cpu_clk),
          .rst_n_in(sys_resetn),
          .rst_n_out(cpu_resetn)
      );
    end else begin : pll_clk
      clk_pll u_clk_pll (
          .cpu_clk(cpu_clk),
          .sys_clk(sys_clk),
          .resetn (~reset),
          .locked (pll_locked),
          .clk_in1(clk)
      );
      rst_sync u_rst_sys (
          .clk(sys_clk),
          .rst_n_in(pll_locked),
          .rst_n_out(sys_resetn)
      );
      rst_sync u_rst_cpu (
          .clk(cpu_clk),
          .rst_n_in(sys_resetn),
          .rst_n_out(cpu_resetn)
      );
    end
  endgenerate


  //--------------------------------------------------------------------------------
  // BUS MATRIX
  //--------------------------------------------------------------------------------
  Axi_CDC u_Axi_CDC (
      .axiInClk      (cpu_clk),
      .axiInRst      (cpu_resetn),
      .axiOutClk     (sys_clk),
      .axiOutRst     (sys_resetn),
      .axiIn_awvalid (cpu_awvalid),
      .axiIn_awready (cpu_awready),
      .axiIn_awaddr  (cpu_awaddr),
      .axiIn_awid    (cpu_awid),
      .axiIn_awlen   (cpu_awlen),
      .axiIn_awsize  (cpu_awsize),
      .axiIn_awburst (cpu_awburst),
      .axiIn_awlock  (cpu_awlock),
      .axiIn_awcache (cpu_awcache),
      .axiIn_awprot  (cpu_awprot),
      .axiIn_wvalid  (cpu_wvalid),
      .axiIn_wready  (cpu_wready),
      .axiIn_wdata   (cpu_wdata),
      .axiIn_wstrb   (cpu_wstrb),
      .axiIn_wlast   (cpu_wlast),
      .axiIn_bvalid  (cpu_bvalid),
      .axiIn_bready  (cpu_bready),
      .axiIn_bid     (cpu_bid),
      .axiIn_bresp   (cpu_bresp),
      .axiIn_arvalid (cpu_arvalid),
      .axiIn_arready (cpu_arready),
      .axiIn_araddr  (cpu_araddr),
      .axiIn_arid    (cpu_arid),
      .axiIn_arlen   (cpu_arlen),
      .axiIn_arsize  (cpu_arsize),
      .axiIn_arburst (cpu_arburst),
      .axiIn_arlock  (cpu_arlock),
      .axiIn_arcache (cpu_arcache),
      .axiIn_arprot  (cpu_arprot),
      .axiIn_rvalid  (cpu_rvalid),
      .axiIn_rready  (cpu_rready),
      .axiIn_rdata   (cpu_rdata),
      .axiIn_rid     (cpu_rid),
      .axiIn_rresp   (cpu_rresp),
      .axiIn_rlast   (cpu_rlast),
      .axiOut_awvalid(axi_mtx_mst0_awvalid),
      .axiOut_awready(axi_mtx_mst0_awready),
      .axiOut_awaddr (axi_mtx_mst0_awaddr),
      .axiOut_awid   (axi_mtx_mst0_awid),
      .axiOut_awlen  (axi_mtx_mst0_awlen),
      .axiOut_awsize (axi_mtx_mst0_awsize),
      .axiOut_awburst(axi_mtx_mst0_awburst),
      .axiOut_awlock (axi_mtx_mst0_awlock),
      .axiOut_awcache(axi_mtx_mst0_awcache),
      .axiOut_awprot (axi_mtx_mst0_awprot),
      .axiOut_wvalid (axi_mtx_mst0_wvalid),
      .axiOut_wready (axi_mtx_mst0_wready),
      .axiOut_wdata  (axi_mtx_mst0_wdata),
      .axiOut_wstrb  (axi_mtx_mst0_wstrb),
      .axiOut_wlast  (axi_mtx_mst0_wlast),
      .axiOut_bvalid (axi_mtx_mst0_bvalid),
      .axiOut_bready (axi_mtx_mst0_bready),
      .axiOut_bid    (axi_mtx_mst0_bid),
      .axiOut_bresp  (axi_mtx_mst0_bresp),
      .axiOut_arvalid(axi_mtx_mst0_arvalid),
      .axiOut_arready(axi_mtx_mst0_arready),
      .axiOut_araddr (axi_mtx_mst0_araddr),
      .axiOut_arid   (axi_mtx_mst0_arid),
      .axiOut_arlen  (axi_mtx_mst0_arlen),
      .axiOut_arsize (axi_mtx_mst0_arsize),
      .axiOut_arburst(axi_mtx_mst0_arburst),
      .axiOut_arlock (axi_mtx_mst0_arlock),
      .axiOut_arcache(axi_mtx_mst0_arcache),
      .axiOut_arprot (axi_mtx_mst0_arprot),
      .axiOut_rvalid (axi_mtx_mst0_rvalid),
      .axiOut_rready (axi_mtx_mst0_rready),
      .axiOut_rdata  (axi_mtx_mst0_rdata),
      .axiOut_rid    (axi_mtx_mst0_rid),
      .axiOut_rresp  (axi_mtx_mst0_rresp),
      .axiOut_rlast  (axi_mtx_mst0_rlast)
  );

  axi_interconnect #(
      .S_COUNT        (2),
      .M_COUNT        (4),
      .DATA_WIDTH     (32),
      .ADDR_WIDTH     (32),
      .ID_WIDTH       (4),
      .FORWARD_ID     (0),
      .M_REGIONS      (1),
      .M_BASE_ADDR    ({32'h1F20_0000, 32'h1F10_0000, 32'h1F00_0000, 32'h1C00_0000}),
      .M_ADDR_WIDTH   ({32'd20, 32'd20, 32'd20, 32'd23}),
      .M_CONNECT_READ ({{1'b1, 1'b1, 1'b1, 1'b1}, {1'b1, 1'b1, 1'b1, 1'b1}}),
      .M_CONNECT_WRITE({{1'b1, 1'b1, 1'b1, 1'b1}, {1'b1, 1'b1, 1'b1, 1'b1}})
  ) u_axi_interconnect (
      .clk           (sys_clk),
      .rst           (~sys_resetn),
      .s_axi_awid    (s_axi_awid),
      .s_axi_awaddr  (s_axi_awaddr),
      .s_axi_awlen   (s_axi_awlen),
      .s_axi_awsize  (s_axi_awsize),
      .s_axi_awburst (s_axi_awburst),
      .s_axi_awlock  (s_axi_awlock),
      .s_axi_awcache (s_axi_awcache),
      .s_axi_awprot  (s_axi_awprot),
      .s_axi_awqos   (s_axi_awqos),
      .s_axi_awuser  (),
      .s_axi_awvalid (s_axi_awvalid),
      .s_axi_awready (s_axi_awready),
      .s_axi_wdata   (s_axi_wdata),
      .s_axi_wstrb   (s_axi_wstrb),
      .s_axi_wlast   (s_axi_wlast),
      .s_axi_wuser   (),
      .s_axi_wvalid  (s_axi_wvalid),
      .s_axi_wready  (s_axi_wready),
      .s_axi_bid     (s_axi_bid),
      .s_axi_bresp   (s_axi_bresp),
      .s_axi_buser   (),
      .s_axi_bvalid  (s_axi_bvalid),
      .s_axi_bready  (s_axi_bready),
      .s_axi_arid    (s_axi_arid),
      .s_axi_araddr  (s_axi_araddr),
      .s_axi_arlen   (s_axi_arlen),
      .s_axi_arsize  (s_axi_arsize),
      .s_axi_arburst (s_axi_arburst),
      .s_axi_arlock  (s_axi_arlock),
      .s_axi_arcache (s_axi_arcache),
      .s_axi_arprot  (s_axi_arprot),
      .s_axi_arqos   (s_axi_arqos),
      .s_axi_aruser  (),
      .s_axi_arvalid (s_axi_arvalid),
      .s_axi_arready (s_axi_arready),
      .s_axi_rid     (s_axi_rid),
      .s_axi_rdata   (s_axi_rdata),
      .s_axi_rresp   (s_axi_rresp),
      .s_axi_rlast   (s_axi_rlast),
      .s_axi_ruser   (),
      .s_axi_rvalid  (s_axi_rvalid),
      .s_axi_rready  (s_axi_rready),
      .m_axi_awid    (),
      .m_axi_awaddr  (m_axi_awaddr),
      .m_axi_awlen   (m_axi_awlen),
      .m_axi_awsize  (m_axi_awsize),
      .m_axi_awburst (m_axi_awburst),
      .m_axi_awlock  (m_axi_awlock),
      .m_axi_awcache (m_axi_awcache),
      .m_axi_awprot  (m_axi_awprot),
      .m_axi_awqos   (m_axi_awqos),
      .m_axi_awregion(m_axi_awregion),
      .m_axi_awuser  (),
      .m_axi_awvalid (m_axi_awvalid),
      .m_axi_awready (m_axi_awready),
      .m_axi_wdata   (m_axi_wdata),
      .m_axi_wstrb   (m_axi_wstrb),
      .m_axi_wlast   (m_axi_wlast),
      .m_axi_wuser   (),
      .m_axi_wvalid  (m_axi_wvalid),
      .m_axi_wready  (m_axi_wready),
      .m_axi_bid     (16'b0),
      .m_axi_bresp   (m_axi_bresp),
      .m_axi_buser   (),
      .m_axi_bvalid  (m_axi_bvalid),
      .m_axi_bready  (m_axi_bready),
      .m_axi_arid    (),
      .m_axi_araddr  (m_axi_araddr),
      .m_axi_arlen   (m_axi_arlen),
      .m_axi_arsize  (m_axi_arsize),
      .m_axi_arburst (m_axi_arburst),
      .m_axi_arlock  (m_axi_arlock),
      .m_axi_arcache (m_axi_arcache),
      .m_axi_arprot  (m_axi_arprot),
      .m_axi_arqos   (m_axi_arqos),
      .m_axi_arregion(m_axi_arregion),
      .m_axi_aruser  (),
      .m_axi_arvalid (m_axi_arvalid),
      .m_axi_arready (m_axi_arready),
      .m_axi_rid     (16'b0),
      .m_axi_rdata   (m_axi_rdata),
      .m_axi_rresp   (m_axi_rresp),
      .m_axi_rlast   (m_axi_rlast),
      .m_axi_ruser   (),
      .m_axi_rvalid  (m_axi_rvalid),
      .m_axi_rready  (m_axi_rready)
  );

  // master input
  assign s_axi_awid = {axi_mtx_mst1_awid, axi_mtx_mst0_awid};
  assign s_axi_awaddr = {axi_mtx_mst1_awaddr, axi_mtx_mst0_awaddr};
  assign s_axi_awlen = {axi_mtx_mst1_awlen, axi_mtx_mst0_awlen};
  assign s_axi_awsize = {axi_mtx_mst1_awsize, axi_mtx_mst0_awsize};
  assign s_axi_awburst = {axi_mtx_mst1_awburst, axi_mtx_mst0_awburst};
  assign s_axi_awlock = {axi_mtx_mst1_awlock, axi_mtx_mst0_awlock};
  assign s_axi_awcache = {axi_mtx_mst1_awcache, axi_mtx_mst0_awcache};
  assign s_axi_awprot = {axi_mtx_mst1_awprot, axi_mtx_mst0_awprot};
  assign s_axi_awqos = {axi_mtx_mst1_awqos, axi_mtx_mst0_awqos};
  assign s_axi_awvalid = {axi_mtx_mst1_awvalid, axi_mtx_mst0_awvalid};
  assign s_axi_wdata = {axi_mtx_mst1_wdata, axi_mtx_mst0_wdata};
  assign s_axi_wstrb = {axi_mtx_mst1_wstrb, axi_mtx_mst0_wstrb};
  assign s_axi_wlast = {axi_mtx_mst1_wlast, axi_mtx_mst0_wlast};
  assign s_axi_wvalid = {axi_mtx_mst1_wvalid, axi_mtx_mst0_wvalid};
  assign s_axi_bready = {axi_mtx_mst1_bready, axi_mtx_mst0_bready};
  assign s_axi_arid = {axi_mtx_mst1_arid, axi_mtx_mst0_arid};
  assign s_axi_araddr = {axi_mtx_mst1_araddr, axi_mtx_mst0_araddr};
  assign s_axi_arlen = {axi_mtx_mst1_arlen, axi_mtx_mst0_arlen};
  assign s_axi_arsize = {axi_mtx_mst1_arsize, axi_mtx_mst0_arsize};
  assign s_axi_arburst = {axi_mtx_mst1_arburst, axi_mtx_mst0_arburst};
  assign s_axi_arlock = {axi_mtx_mst1_arlock, axi_mtx_mst0_arlock};
  assign s_axi_arcache = {axi_mtx_mst1_arcache, axi_mtx_mst0_arcache};
  assign s_axi_arprot = {axi_mtx_mst1_arprot, axi_mtx_mst0_arprot};
  assign s_axi_arqos = {axi_mtx_mst1_arqos, axi_mtx_mst0_arqos};
  assign s_axi_arvalid = {axi_mtx_mst1_arvalid, axi_mtx_mst0_arvalid};
  assign s_axi_rready = {axi_mtx_mst1_rready, axi_mtx_mst0_rready};

  // master output
  assign {axi_mtx_mst1_awready, axi_mtx_mst0_awready} = s_axi_awready;
  assign {axi_mtx_mst1_wready, axi_mtx_mst0_wready} = s_axi_wready;
  assign {axi_mtx_mst1_bid, axi_mtx_mst0_bid} = s_axi_bid;
  assign {axi_mtx_mst1_bresp, axi_mtx_mst0_bresp} = s_axi_bresp;
  assign {axi_mtx_mst1_bvalid, axi_mtx_mst0_bvalid} = s_axi_bvalid;
  assign {axi_mtx_mst1_arready, axi_mtx_mst0_arready} = s_axi_arready;
  assign {axi_mtx_mst1_rid, axi_mtx_mst0_rid} = s_axi_rid;
  assign {axi_mtx_mst1_rdata, axi_mtx_mst0_rdata} = s_axi_rdata;
  assign {axi_mtx_mst1_rresp, axi_mtx_mst0_rresp} = s_axi_rresp;
  assign {axi_mtx_mst1_rlast, axi_mtx_mst0_rlast} = s_axi_rlast;
  assign {axi_mtx_mst1_rvalid, axi_mtx_mst0_rvalid} = s_axi_rvalid;

  // slave input
  assign m_axi_awready = {
    axi_mtx_slv3_awready, axi_mtx_slv2_awready, axi_mtx_slv1_awready, axi_mtx_slv0_awready
  };
  assign m_axi_wready = {
    axi_mtx_slv3_wready, axi_mtx_slv2_wready, axi_mtx_slv1_wready, axi_mtx_slv0_wready
  };
  assign m_axi_bresp = {
    axi_mtx_slv3_bresp, axi_mtx_slv2_bresp, axi_mtx_slv1_bresp, axi_mtx_slv0_bresp
  };
  assign m_axi_bvalid = {
    axi_mtx_slv3_bvalid, axi_mtx_slv2_bvalid, axi_mtx_slv1_bvalid, axi_mtx_slv0_bvalid
  };
  assign m_axi_arready = {
    axi_mtx_slv3_arready, axi_mtx_slv2_arready, axi_mtx_slv1_arready, axi_mtx_slv0_arready
  };
  assign m_axi_rdata = {
    axi_mtx_slv3_rdata, axi_mtx_slv2_rdata, axi_mtx_slv1_rdata, axi_mtx_slv0_rdata
  };
  assign m_axi_rresp = {
    axi_mtx_slv3_rresp, axi_mtx_slv2_rresp, axi_mtx_slv1_rresp, axi_mtx_slv0_rresp
  };
  assign m_axi_rlast = {
    axi_mtx_slv3_rlast, axi_mtx_slv2_rlast, axi_mtx_slv1_rlast, axi_mtx_slv0_rlast
  };
  assign m_axi_rvalid = {
    axi_mtx_slv3_rvalid, axi_mtx_slv2_rvalid, axi_mtx_slv1_rvalid, axi_mtx_slv0_rvalid
  };

  // slave output
  assign {axi_mtx_slv3_awaddr, axi_mtx_slv2_awaddr, axi_mtx_slv1_awaddr, axi_mtx_slv0_awaddr} = m_axi_awaddr;
  assign {axi_mtx_slv3_awlen, axi_mtx_slv2_awlen, axi_mtx_slv1_awlen, axi_mtx_slv0_awlen} = m_axi_awlen;
  assign {axi_mtx_slv3_awsize, axi_mtx_slv2_awsize, axi_mtx_slv1_awsize, axi_mtx_slv0_awsize} = m_axi_awsize;
  assign {axi_mtx_slv3_awburst, axi_mtx_slv2_awburst, axi_mtx_slv1_awburst, axi_mtx_slv0_awburst} = m_axi_awburst;
  assign {axi_mtx_slv3_awlock, axi_mtx_slv2_awlock, axi_mtx_slv1_awlock, axi_mtx_slv0_awlock} = m_axi_awlock;
  assign {axi_mtx_slv3_awcache, axi_mtx_slv2_awcache, axi_mtx_slv1_awcache, axi_mtx_slv0_awcache} = m_axi_awcache;
  assign {axi_mtx_slv3_awprot, axi_mtx_slv2_awprot, axi_mtx_slv1_awprot, axi_mtx_slv0_awprot} = m_axi_awprot;
  assign {axi_mtx_slv3_awregion, axi_mtx_slv2_awregion, axi_mtx_slv1_awregion, axi_mtx_slv0_awregion} = m_axi_awregion;
  assign {axi_mtx_slv3_awqos, axi_mtx_slv2_awqos, axi_mtx_slv1_awqos, axi_mtx_slv0_awqos} = m_axi_awqos;
  assign {axi_mtx_slv3_awvalid, axi_mtx_slv2_awvalid, axi_mtx_slv1_awvalid, axi_mtx_slv0_awvalid} = m_axi_awvalid;
  assign {axi_mtx_slv3_wdata, axi_mtx_slv2_wdata, axi_mtx_slv1_wdata, axi_mtx_slv0_wdata} = m_axi_wdata;
  assign {axi_mtx_slv3_wstrb, axi_mtx_slv2_wstrb, axi_mtx_slv1_wstrb, axi_mtx_slv0_wstrb} = m_axi_wstrb;
  assign {axi_mtx_slv3_wlast, axi_mtx_slv2_wlast, axi_mtx_slv1_wlast, axi_mtx_slv0_wlast} = m_axi_wlast;
  assign {axi_mtx_slv3_wvalid, axi_mtx_slv2_wvalid, axi_mtx_slv1_wvalid, axi_mtx_slv0_wvalid} = m_axi_wvalid;
  assign {axi_mtx_slv3_bready, axi_mtx_slv2_bready, axi_mtx_slv1_bready, axi_mtx_slv0_bready} = m_axi_bready;
  assign {axi_mtx_slv3_araddr, axi_mtx_slv2_araddr, axi_mtx_slv1_araddr, axi_mtx_slv0_araddr} = m_axi_araddr;
  assign {axi_mtx_slv3_arlen, axi_mtx_slv2_arlen, axi_mtx_slv1_arlen, axi_mtx_slv0_arlen} = m_axi_arlen;
  assign {axi_mtx_slv3_arsize, axi_mtx_slv2_arsize, axi_mtx_slv1_arsize, axi_mtx_slv0_arsize} = m_axi_arsize;
  assign {axi_mtx_slv3_arburst, axi_mtx_slv2_arburst, axi_mtx_slv1_arburst, axi_mtx_slv0_arburst} = m_axi_arburst;
  assign {axi_mtx_slv3_arlock, axi_mtx_slv2_arlock, axi_mtx_slv1_arlock, axi_mtx_slv0_arlock} = m_axi_arlock;
  assign {axi_mtx_slv3_arcache, axi_mtx_slv2_arcache, axi_mtx_slv1_arcache, axi_mtx_slv0_arcache} = m_axi_arcache;
  assign {axi_mtx_slv3_arprot, axi_mtx_slv2_arprot, axi_mtx_slv1_arprot, axi_mtx_slv0_arprot} = m_axi_arprot;
  assign {axi_mtx_slv3_arregion, axi_mtx_slv2_arregion, axi_mtx_slv1_arregion, axi_mtx_slv0_arregion} = m_axi_arregion;
  assign {axi_mtx_slv3_arqos, axi_mtx_slv2_arqos, axi_mtx_slv1_arqos, axi_mtx_slv0_arqos} = m_axi_arqos;
  assign {axi_mtx_slv3_arvalid, axi_mtx_slv2_arvalid, axi_mtx_slv1_arvalid, axi_mtx_slv0_arvalid} = m_axi_arvalid;
  assign {axi_mtx_slv3_rready, axi_mtx_slv2_rready, axi_mtx_slv1_rready, axi_mtx_slv0_rready} = m_axi_rready;

  // BUS MATRIX slave unused signals (SASD)
  assign {axi_mtx_slv3_awid, axi_mtx_slv2_awid, axi_mtx_slv1_awid, axi_mtx_slv0_awid} = 20'b0;
  assign {axi_mtx_slv3_arid, axi_mtx_slv2_arid, axi_mtx_slv1_arid, axi_mtx_slv0_arid} = 20'b0;


  //--------------------------------------------------------------------------------
  // MASTER0, CPU
  //--------------------------------------------------------------------------------
  core_top #(
      .TLBNUM(32)
  ) u_core_top (
      .aclk              (cpu_clk),
      .aresetn           (cpu_resetn),
      .intrpt            (intrpt),
      .arid              (cpu_arid),
      .araddr            (cpu_araddr),
      .arlen             (cpu_arlen),
      .arsize            (cpu_arsize),
      .arburst           (cpu_arburst),
      .arlock            (cpu_arlock),
      .arcache           (cpu_arcache),
      .arprot            (cpu_arprot),
      .arvalid           (cpu_arvalid),
      .arready           (cpu_arready),
      .rid               (cpu_rid),
      .rdata             (cpu_rdata),
      .rresp             (cpu_rresp),
      .rlast             (cpu_rlast),
      .rvalid            (cpu_rvalid),
      .rready            (cpu_rready),
      .awid              (cpu_awid),
      .awaddr            (cpu_awaddr),
      .awlen             (cpu_awlen),
      .awsize            (cpu_awsize),
      .awburst           (cpu_awburst),
      .awlock            (cpu_awlock),
      .awcache           (cpu_awcache),
      .awprot            (cpu_awprot),
      .awvalid           (cpu_awvalid),
      .awready           (cpu_awready),
      .wid               (),
      .wdata             (cpu_wdata),
      .wstrb             (cpu_wstrb),
      .wlast             (cpu_wlast),
      .wvalid            (cpu_wvalid),
      .wready            (cpu_wready),
      .bid               (cpu_bid),
      .bresp             (cpu_bresp),
      .bvalid            (cpu_bvalid),
      .bready            (cpu_bready),
      .break_point       (1'b0),
      .infor_flag        (1'b0),
      .reg_num           (5'b0),
      .ws_valid          (),
      .rf_rdata          (),
      .debug0_wb_pc      (debug0_wb_pc),
      .debug0_wb_rf_wen  (debug0_wb_rf_wen),
      .debug0_wb_rf_wnum (debug0_wb_rf_wnum),
      .debug0_wb_rf_wdata(debug0_wb_rf_wdata),
      .debug0_wb_inst    (debug0_wb_inst)
  );

  assign intrpt[7:0] = {
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0,  // Reserved
    1'b0  // Reserved
  };

  // CPU AXI unused signals
  assign axi_mtx_mst0_awqos = 4'b0;
  assign axi_mtx_mst0_arqos = 4'b0;


  //--------------------------------------------------------------------------------
  // MASTER1, DMAC
  //--------------------------------------------------------------------------------
  dma_axi32 u_dmac (
      .clk          (sys_clk),
      .reset        (~sys_resetn),
      .scan_en      (1'b0),
      .idle         (),
      .INT          (dmac_int),
      .periph_tx_req(periph_tx_req),
      .periph_tx_clr(periph_tx_clr),
      .periph_rx_req(periph_rx_req),
      .periph_rx_clr(periph_rx_clr),
      .pclken       (dmac_pclken),
      .psel         (dmac_psel),
      .penable      (dmac_penable),
      .paddr        (dmac_paddr),
      .pwrite       (dmac_pwrite),
      .pwdata       (dmac_pwdata),
      .prdata       (dmac_prdata),
      .pslverr      (dmac_pslverr),
      .pready       (dmac_pready),
      .AWID0        (axi_mtx_mst1_awid[3]),
      .AWADDR0      (axi_mtx_mst1_awaddr),
      .AWLEN0       (axi_mtx_mst1_awlen[3:0]),
      .AWSIZE0      (axi_mtx_mst1_awsize[1:0]),
      .AWVALID0     (axi_mtx_mst1_awvalid),
      .AWREADY0     (axi_mtx_mst1_awready),
      .WID0         (),
      .WDATA0       (axi_mtx_mst1_wdata),
      .WSTRB0       (axi_mtx_mst1_wstrb),
      .WLAST0       (axi_mtx_mst1_wlast),
      .WVALID0      (axi_mtx_mst1_wvalid),
      .WREADY0      (axi_mtx_mst1_wready),
      .BID0         (axi_mtx_mst1_bid[3]),
      .BRESP0       (axi_mtx_mst1_bresp),
      .BVALID0      (axi_mtx_mst1_bvalid),
      .BREADY0      (axi_mtx_mst1_bready),
      .ARID0        (axi_mtx_mst1_arid[3]),
      .ARADDR0      (axi_mtx_mst1_araddr),
      .ARLEN0       (axi_mtx_mst1_arlen[3:0]),
      .ARSIZE0      (axi_mtx_mst1_arsize[1:0]),
      .ARVALID0     (axi_mtx_mst1_arvalid),
      .ARREADY0     (axi_mtx_mst1_arready),
      .RID0         (axi_mtx_mst1_rid[3]),
      .RDATA0       (axi_mtx_mst1_rdata),
      .RRESP0       (axi_mtx_mst1_rresp),
      .RLAST0       (axi_mtx_mst1_rlast),
      .RVALID0      (axi_mtx_mst1_rvalid),
      .RREADY0      (axi_mtx_mst1_rready)
  );

  // DMAC hardware req/ack
  assign periph_tx_req = 31'b0;
  assign periph_rx_req = 31'b0;

  // DMAC AXI ID
  assign axi_mtx_mst1_awid[2:0] = {3{1'b1}};
  assign axi_mtx_mst1_arid[2:0] = {3{1'b1}};

  // DMAC AXI unused signals
  assign axi_mtx_mst1_awlen[7:4] = 4'b0;
  assign axi_mtx_mst1_arlen[7:4] = 4'b0;
  assign axi_mtx_mst1_awsize[2:2] = 1'b0;
  assign axi_mtx_mst1_arsize[2:2] = 1'b0;
  assign axi_mtx_mst1_awburst = 2'b0;
  assign axi_mtx_mst1_awlock = 1'b0;
  assign axi_mtx_mst1_awcache = 4'b0;
  assign axi_mtx_mst1_awprot = 3'b0;
  assign axi_mtx_mst1_arburst = 2'b0;
  assign axi_mtx_mst1_arlock = 1'b0;
  assign axi_mtx_mst1_arcache = 4'b0;
  assign axi_mtx_mst1_arprot = 3'b0;
  assign axi_mtx_mst1_awqos = 4'b0;
  assign axi_mtx_mst1_arqos = 4'b0;


  //--------------------------------------------------------------------------------
  // SLAVE0, SRAM
  //--------------------------------------------------------------------------------
  axi_wrap_ram_sp_ext u_axi_wrap_ram_sp_ext (
      .aclk         (sys_clk),
      .aresetn      (sys_resetn),
      .axi_arid     (axi_mtx_slv0_arid),
      .axi_araddr   (axi_mtx_slv0_araddr),
      .axi_arlen    (axi_mtx_slv0_arlen),
      .axi_arsize   (axi_mtx_slv0_arsize),
      .axi_arburst  (axi_mtx_slv0_arburst),
      .axi_arlock   ({1'b0, axi_mtx_slv0_arlock}),
      .axi_arcache  (axi_mtx_slv0_arcache),
      .axi_arprot   (axi_mtx_slv0_arprot),
      .axi_arvalid  (axi_mtx_slv0_arvalid),
      .axi_arready  (axi_mtx_slv0_arready),
      .axi_rid      (axi_mtx_slv0_rid),
      .axi_rdata    (axi_mtx_slv0_rdata),
      .axi_rresp    (axi_mtx_slv0_rresp),
      .axi_rlast    (axi_mtx_slv0_rlast),
      .axi_rvalid   (axi_mtx_slv0_rvalid),
      .axi_rready   (axi_mtx_slv0_rready),
      .axi_awid     (axi_mtx_slv0_awid),
      .axi_awaddr   (axi_mtx_slv0_awaddr),
      .axi_awlen    (axi_mtx_slv0_awlen),
      .axi_awsize   (axi_mtx_slv0_awsize),
      .axi_awburst  (axi_mtx_slv0_awburst),
      .axi_awlock   ({1'b0, axi_mtx_slv0_awlock}),
      .axi_awcache  (axi_mtx_slv0_awcache),
      .axi_awprot   (axi_mtx_slv0_awprot),
      .axi_awvalid  (axi_mtx_slv0_awvalid),
      .axi_awready  (axi_mtx_slv0_awready),
      .axi_wdata    (axi_mtx_slv0_wdata),
      .axi_wstrb    (axi_mtx_slv0_wstrb),
      .axi_wlast    (axi_mtx_slv0_wlast),
      .axi_wvalid   (axi_mtx_slv0_wvalid),
      .axi_wready   (axi_mtx_slv0_wready),
      .axi_bid      (axi_mtx_slv0_bid),
      .axi_bresp    (axi_mtx_slv0_bresp),
      .axi_bvalid   (axi_mtx_slv0_bvalid),
      .axi_bready   (axi_mtx_slv0_bready),
      .base_ram_data(base_ram_data),
      .base_ram_addr(base_ram_addr),
      .base_ram_be_n(base_ram_be_n),
      .base_ram_ce_n(base_ram_ce_n),
      .base_ram_oe_n(base_ram_oe_n),
      .base_ram_we_n(base_ram_we_n),
      .ext_ram_data (ext_ram_data),
      .ext_ram_addr (ext_ram_addr),
      .ext_ram_be_n (ext_ram_be_n),
      .ext_ram_ce_n (ext_ram_ce_n),
      .ext_ram_oe_n (ext_ram_oe_n),
      .ext_ram_we_n (ext_ram_we_n)
  );


  //--------------------------------------------------------------------------------
  // SLAVE1, UART
  //--------------------------------------------------------------------------------
  axi_uart_controller u_axi_uart_controller (
      .clk          (sys_clk),
      .rst_n        (sys_resetn),
      .axi_s_awid   (axi_mtx_slv1_awid),
      .axi_s_awaddr (axi_mtx_slv1_awaddr),
      .axi_s_awlen  (axi_mtx_slv1_awlen),
      .axi_s_awsize (axi_mtx_slv1_awsize),
      .axi_s_awburst(axi_mtx_slv1_awburst),
      .axi_s_awlock (axi_mtx_slv1_awlock),
      .axi_s_awcache(axi_mtx_slv1_awcache),
      .axi_s_awprot (axi_mtx_slv1_awprot),
      .axi_s_awvalid(axi_mtx_slv1_awvalid),
      .axi_s_awready(axi_mtx_slv1_awready),
      .axi_s_wid    (5'b0),
      .axi_s_wdata  (axi_mtx_slv1_wdata),
      .axi_s_wstrb  (axi_mtx_slv1_wstrb),
      .axi_s_wlast  (axi_mtx_slv1_wlast),
      .axi_s_wvalid (axi_mtx_slv1_wvalid),
      .axi_s_wready (axi_mtx_slv1_wready),
      .axi_s_bid    (axi_mtx_slv1_bid),
      .axi_s_bresp  (axi_mtx_slv1_bresp),
      .axi_s_bvalid (axi_mtx_slv1_bvalid),
      .axi_s_bready (axi_mtx_slv1_bready),
      .axi_s_arid   (axi_mtx_slv1_arid),
      .axi_s_araddr (axi_mtx_slv1_araddr),
      .axi_s_arlen  (axi_mtx_slv1_arlen),
      .axi_s_arsize (axi_mtx_slv1_arsize),
      .axi_s_arburst(axi_mtx_slv1_arburst),
      .axi_s_arlock (axi_mtx_slv1_arlock),
      .axi_s_arcache(axi_mtx_slv1_arcache),
      .axi_s_arprot (axi_mtx_slv1_arprot),
      .axi_s_arvalid(axi_mtx_slv1_arvalid),
      .axi_s_arready(axi_mtx_slv1_arready),
      .axi_s_rid    (axi_mtx_slv1_rid),
      .axi_s_rdata  (axi_mtx_slv1_rdata),
      .axi_s_rresp  (axi_mtx_slv1_rresp),
      .axi_s_rlast  (axi_mtx_slv1_rlast),
      .axi_s_rvalid (axi_mtx_slv1_rvalid),
      .axi_s_rready (axi_mtx_slv1_rready),
      .apb_rw_dma   (1'b0),
      .apb_psel_dma (1'b0),
      .apb_enab_dma (1'b0),
      .apb_addr_dma (20'b0),
      .apb_valid_dma(1'b0),
      .apb_wdata_dma(32'b0),
      .apb_rdata_dma(),
      .apb_ready_dma(),
      .dma_grant    (),
      .dma_req_o    (),
      .dma_ack_i    (1'b0),
      .uart0_txd_i  (uart0_txd_i),
      .uart0_txd_o  (uart0_txd_o),
      .uart0_txd_oe (uart0_txd_oe),
      .uart0_rxd_i  (uart0_rxd_i),
      .uart0_rxd_o  (uart0_rxd_o),
      .uart0_rxd_oe (uart0_rxd_oe),
      .uart0_rts_o  (uart0_rts_o),
      .uart0_dtr_o  (uart0_dtr_o),
      .uart0_cts_i  (uart0_cts_i),
      .uart0_dsr_i  (uart0_dsr_i),
      .uart0_dcd_i  (uart0_dcd_i),
      .uart0_ri_i   (uart0_ri_i),
      .uart0_int    (uart0_int)
  );

  assign UART_CTS = 1'b0;
  assign UART_DSR = 1'b0;
  assign UART_DCD = 1'b0;
  assign UART_RI = 1'b0;
  assign UART_RX = uart0_rxd_oe ? 1'bz : uart0_rxd_o;
  assign UART_TX = uart0_txd_oe ? 1'bz : uart0_txd_o;
  assign UART_RTS = uart0_rts_o;
  assign UART_DTR = uart0_dtr_o;
  assign uart0_txd_i = UART_TX;
  assign uart0_rxd_i = UART_RX;
  assign uart0_cts_i = UART_CTS;
  assign uart0_dcd_i = UART_DCD;
  assign uart0_dsr_i = UART_DSR;
  assign uart0_ri_i = UART_RI;


  //--------------------------------------------------------------------------------
  // SLAVE2, X2P
  //--------------------------------------------------------------------------------
  axi_axil_adapter #(
      .ADDR_WIDTH     (32),
      .AXI_DATA_WIDTH (32),
      .AXI_ID_WIDTH   (5),
      .AXIL_DATA_WIDTH(32)
  ) u_x2xl (
      .clk           (sys_clk),
      .rst           (~sys_resetn),
      .s_axi_awid    (axi_mtx_slv2_awid),
      .s_axi_awaddr  (axi_mtx_slv2_awaddr),
      .s_axi_awlen   (axi_mtx_slv2_awlen),
      .s_axi_awsize  (axi_mtx_slv2_awsize),
      .s_axi_awburst (axi_mtx_slv2_awburst),
      .s_axi_awlock  (axi_mtx_slv2_awlock),
      .s_axi_awcache (axi_mtx_slv2_awcache),
      .s_axi_awprot  (axi_mtx_slv2_awprot),
      .s_axi_awvalid (axi_mtx_slv2_awvalid),
      .s_axi_awready (axi_mtx_slv2_awready),
      .s_axi_wdata   (axi_mtx_slv2_wdata),
      .s_axi_wstrb   (axi_mtx_slv2_wstrb),
      .s_axi_wlast   (axi_mtx_slv2_wlast),
      .s_axi_wvalid  (axi_mtx_slv2_wvalid),
      .s_axi_wready  (axi_mtx_slv2_wready),
      .s_axi_bid     (axi_mtx_slv2_bid),
      .s_axi_bresp   (axi_mtx_slv2_bresp),
      .s_axi_bvalid  (axi_mtx_slv2_bvalid),
      .s_axi_bready  (axi_mtx_slv2_bready),
      .s_axi_arid    (axi_mtx_slv2_arid),
      .s_axi_araddr  (axi_mtx_slv2_araddr),
      .s_axi_arlen   (axi_mtx_slv2_arlen),
      .s_axi_arsize  (axi_mtx_slv2_arsize),
      .s_axi_arburst (axi_mtx_slv2_arburst),
      .s_axi_arlock  (axi_mtx_slv2_arlock),
      .s_axi_arcache (axi_mtx_slv2_arcache),
      .s_axi_arprot  (axi_mtx_slv2_arprot),
      .s_axi_arvalid (axi_mtx_slv2_arvalid),
      .s_axi_arready (axi_mtx_slv2_arready),
      .s_axi_rid     (axi_mtx_slv2_rid),
      .s_axi_rdata   (axi_mtx_slv2_rdata),
      .s_axi_rresp   (axi_mtx_slv2_rresp),
      .s_axi_rlast   (axi_mtx_slv2_rlast),
      .s_axi_rvalid  (axi_mtx_slv2_rvalid),
      .s_axi_rready  (axi_mtx_slv2_rready),
      .m_axil_awaddr (x2p_axi_awaddr),
      .m_axil_awprot (),
      .m_axil_awvalid(x2p_axi_awvalid),
      .m_axil_awready(x2p_axi_awready),
      .m_axil_wdata  (x2p_axi_wdata),
      .m_axil_wstrb  (),
      .m_axil_wvalid (x2p_axi_wvalid),
      .m_axil_wready (x2p_axi_wready),
      .m_axil_bresp  (x2p_axi_bresp),
      .m_axil_bvalid (x2p_axi_bvalid),
      .m_axil_bready (x2p_axi_bready),
      .m_axil_araddr (x2p_axi_araddr),
      .m_axil_arprot (),
      .m_axil_arvalid(x2p_axi_arvalid),
      .m_axil_arready(x2p_axi_arready),
      .m_axil_rdata  (x2p_axi_rdata),
      .m_axil_rresp  (x2p_axi_rresp),
      .m_axil_rvalid (x2p_axi_rvalid),
      .m_axil_rready (x2p_axi_rready)
  );

  x2p u_x2p (
      .s_axi_aclk   (sys_clk),          // input wire s_axi_aclk
      .s_axi_aresetn(sys_resetn),       // input wire s_axi_aresetn
      .s_axi_awaddr (x2p_axi_awaddr),   // input wire [31 : 0] s_axi_awaddr
      .s_axi_awvalid(x2p_axi_awvalid),  // input wire s_axi_awvalid
      .s_axi_awready(x2p_axi_awready),  // output wire s_axi_awready
      .s_axi_wdata  (x2p_axi_wdata),    // input wire [31 : 0] s_axi_wdata
      .s_axi_wvalid (x2p_axi_wvalid),   // input wire s_axi_wvalid
      .s_axi_wready (x2p_axi_wready),   // output wire s_axi_wready
      .s_axi_bresp  (x2p_axi_bresp),    // output wire [1 : 0] s_axi_bresp
      .s_axi_bvalid (x2p_axi_bvalid),   // output wire s_axi_bvalid
      .s_axi_bready (x2p_axi_bready),   // input wire s_axi_bready
      .s_axi_araddr (x2p_axi_araddr),   // input wire [31 : 0] s_axi_araddr
      .s_axi_arvalid(x2p_axi_arvalid),  // input wire s_axi_arvalid
      .s_axi_arready(x2p_axi_arready),  // output wire s_axi_arready
      .s_axi_rdata  (x2p_axi_rdata),    // output wire [31 : 0] s_axi_rdata
      .s_axi_rresp  (x2p_axi_rresp),    // output wire [1 : 0] s_axi_rresp
      .s_axi_rvalid (x2p_axi_rvalid),   // output wire s_axi_rvalid
      .s_axi_rready (x2p_axi_rready),   // input wire s_axi_rready
      .m_apb_paddr  (x2p_apb_paddr),    // output wire [31 : 0] m_apb_paddr
      .m_apb_psel   (x2p_apb_psel),     // output wire [3 : 0] m_apb_psel
      .m_apb_penable(x2p_apb_penable),  // output wire m_apb_penable
      .m_apb_pwrite (x2p_apb_pwrite),   // output wire m_apb_pwrite
      .m_apb_pwdata (x2p_apb_pwdata),   // output wire [31 : 0] m_apb_pwdata
      .m_apb_pready (x2p_apb_pready),   // input wire [3 : 0] m_apb_pready
      .m_apb_prdata (x2p_apb_prdata),   // input wire [31 : 0] m_apb_prdata
      .m_apb_prdata2(x2p_apb_prdata2),  // input wire [31 : 0] m_apb_prdata2
      .m_apb_prdata3(x2p_apb_prdata3),  // input wire [31 : 0] m_apb_prdata3
      .m_apb_prdata4(x2p_apb_prdata4),  // input wire [31 : 0] m_apb_prdata4
      .m_apb_pslverr(x2p_apb_pslverr)   // input wire [3 : 0] m_apb_pslverr
  );


  //--------------------------------------------------------------------------------
  // SLAVE3, CONFREG
  //--------------------------------------------------------------------------------
  confreg #(
      .SIMULATION(SIMULATION)
  ) u_confreg (
      .aclk       (sys_clk),
      .aresetn    (sys_resetn),
      .cpu_clk    (cpu_clk),
      .cpu_resetn (cpu_resetn),
      .s_awid     (axi_mtx_slv3_awid),
      .s_awaddr   (axi_mtx_slv3_awaddr),
      .s_awlen    (axi_mtx_slv3_awlen),
      .s_awsize   (axi_mtx_slv3_awsize),
      .s_awburst  (axi_mtx_slv3_awburst),
      .s_awlock   (axi_mtx_slv3_awlock),
      .s_awcache  (axi_mtx_slv3_awcache),
      .s_awprot   (axi_mtx_slv3_awprot),
      .s_awvalid  (axi_mtx_slv3_awvalid),
      .s_awready  (axi_mtx_slv3_awready),
      .s_wid      (5'b0),
      .s_wdata    (axi_mtx_slv3_wdata),
      .s_wstrb    (axi_mtx_slv3_wstrb),
      .s_wlast    (axi_mtx_slv3_wlast),
      .s_wvalid   (axi_mtx_slv3_wvalid),
      .s_wready   (axi_mtx_slv3_wready),
      .s_bid      (axi_mtx_slv3_bid),
      .s_bresp    (axi_mtx_slv3_bresp),
      .s_bvalid   (axi_mtx_slv3_bvalid),
      .s_bready   (axi_mtx_slv3_bready),
      .s_arid     (axi_mtx_slv3_arid),
      .s_araddr   (axi_mtx_slv3_araddr),
      .s_arlen    (axi_mtx_slv3_arlen),
      .s_arsize   (axi_mtx_slv3_arsize),
      .s_arburst  (axi_mtx_slv3_arburst),
      .s_arlock   (axi_mtx_slv3_arlock),
      .s_arcache  (axi_mtx_slv3_arcache),
      .s_arprot   (axi_mtx_slv3_arprot),
      .s_arvalid  (axi_mtx_slv3_arvalid),
      .s_arready  (axi_mtx_slv3_arready),
      .s_rid      (axi_mtx_slv3_rid),
      .s_rdata    (axi_mtx_slv3_rdata),
      .s_rresp    (axi_mtx_slv3_rresp),
      .s_rlast    (axi_mtx_slv3_rlast),
      .s_rvalid   (axi_mtx_slv3_rvalid),
      .s_rready   (axi_mtx_slv3_rready),
      .led        (leds),
      .dpy0       (dpy0),
      .dpy1       (dpy1),
      .switch     (dip_sw),
      .touch_btn  (touch_btn),
      .confreg_int(confreg_int)
  );


  //--------------------------------------------------------------------------------
  // APB SLAVE0, RESERVED
  //--------------------------------------------------------------------------------
  assign x2p_apb_prdata = 32'b0;
  assign x2p_apb_pready[0] = 1'b1;
  assign x2p_apb_pslverr[0] = 1'b0;


  //--------------------------------------------------------------------------------
  // APB SLAVE1, DMAC REGS
  //--------------------------------------------------------------------------------
  assign dmac_pclken = 1'b1;
  assign dmac_psel = x2p_apb_psel[1];
  assign dmac_penable = x2p_apb_penable;
  assign dmac_paddr = x2p_apb_paddr[12:0];
  assign dmac_pwrite = x2p_apb_pwrite;
  assign dmac_pwdata = x2p_apb_pwdata;
  assign x2p_apb_prdata2 = dmac_prdata;
  assign x2p_apb_pready[1] = dmac_pready;
  assign x2p_apb_pslverr[1] = dmac_pslverr;


  //--------------------------------------------------------------------------------
  // APB SLAVE2, RESERVED
  //--------------------------------------------------------------------------------
  assign x2p_apb_prdata3 = 32'b0;
  assign x2p_apb_pready[2] = 1'b1;
  assign x2p_apb_pslverr[2] = 1'b0;


  //--------------------------------------------------------------------------------
  // APB SLAVE3, RESERVED
  //--------------------------------------------------------------------------------
  // assign x2p_apb_prdata4 = 32'b0;
  // assign x2p_apb_pready[3] = 1'b1;
  // assign x2p_apb_pslverr[3] = 1'b0;

  // dummy slave
  reg [31:0] prdata;
  reg pready;
  always @(posedge sys_clk or negedge sys_resetn) begin
    if (!sys_resetn) begin
      prdata <= 32'b0;
      pready <= 1'b1;
    end else begin
      if (x2p_apb_paddr == 32'h1F130000 && x2p_apb_psel[3]) begin
        prdata <= 32'hDEADBEEF;
        pready <= 1'b1;
      end else begin
        prdata <= 32'b0;
        pready <= 1'b1;
      end
    end
  end
  assign x2p_apb_prdata4 = prdata;
  assign x2p_apb_pready[3] = pready;
  assign x2p_apb_pslverr[3] = 1'b0;


endmodule

