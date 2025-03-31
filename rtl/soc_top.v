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
  wire        cpu_clk;
  wire        cpu_resetn;
  wire        sys_clk;
  wire        sys_resetn;
  //--------------------------------------------------------------------------------
  // CPU
  //--------------------------------------------------------------------------------
  wire [ 7:0] intrpt;
  wire [31:0] debug0_wb_pc;
  wire [ 3:0] debug0_wb_rf_wen;
  wire [ 4:0] debug0_wb_rf_wnum;
  wire [31:0] debug0_wb_rf_wdata;
  wire [31:0] debug0_wb_inst;
  //--------------------------------------------------------------------------------
  // BUS MATRIX
  //--------------------------------------------------------------------------------
  wire        axi_cdc_i_awvalid;
  wire        axi_cdc_i_awready;
  wire [31:0] axi_cdc_i_awaddr;
  wire [ 3:0] axi_cdc_i_awid;
  wire [ 7:0] axi_cdc_i_awlen;
  wire [ 2:0] axi_cdc_i_awsize;
  wire [ 1:0] axi_cdc_i_awburst;
  wire [ 0:0] axi_cdc_i_awlock;
  wire [ 3:0] axi_cdc_i_awcache;
  wire [ 2:0] axi_cdc_i_awprot;
  wire        axi_cdc_i_wvalid;
  wire        axi_cdc_i_wready;
  wire [31:0] axi_cdc_i_wdata;
  wire [ 3:0] axi_cdc_i_wstrb;
  wire        axi_cdc_i_wlast;
  wire        axi_cdc_i_bvalid;
  wire        axi_cdc_i_bready;
  wire [ 3:0] axi_cdc_i_bid;
  wire [ 1:0] axi_cdc_i_bresp;
  wire        axi_cdc_i_arvalid;
  wire        axi_cdc_i_arready;
  wire [31:0] axi_cdc_i_araddr;
  wire [ 3:0] axi_cdc_i_arid;
  wire [ 7:0] axi_cdc_i_arlen;
  wire [ 2:0] axi_cdc_i_arsize;
  wire [ 1:0] axi_cdc_i_arburst;
  wire [ 0:0] axi_cdc_i_arlock;
  wire [ 3:0] axi_cdc_i_arcache;
  wire [ 2:0] axi_cdc_i_arprot;
  wire        axi_cdc_i_rvalid;
  wire        axi_cdc_i_rready;
  wire [31:0] axi_cdc_i_rdata;
  wire [ 3:0] axi_cdc_i_rid;
  wire [ 1:0] axi_cdc_i_rresp;
  wire        axi_cdc_i_rlast;
  wire        axi_cdc_o_awvalid;
  wire        axi_cdc_o_awready;
  wire [31:0] axi_cdc_o_awaddr;
  wire [ 3:0] axi_cdc_o_awid;
  wire [ 7:0] axi_cdc_o_awlen;
  wire [ 2:0] axi_cdc_o_awsize;
  wire [ 1:0] axi_cdc_o_awburst;
  wire [ 0:0] axi_cdc_o_awlock;
  wire [ 3:0] axi_cdc_o_awcache;
  wire [ 2:0] axi_cdc_o_awprot;
  wire        axi_cdc_o_wvalid;
  wire        axi_cdc_o_wready;
  wire [31:0] axi_cdc_o_wdata;
  wire [ 3:0] axi_cdc_o_wstrb;
  wire        axi_cdc_o_wlast;
  wire        axi_cdc_o_bvalid;
  wire        axi_cdc_o_bready;
  wire [ 3:0] axi_cdc_o_bid;
  wire [ 1:0] axi_cdc_o_bresp;
  wire        axi_cdc_o_arvalid;
  wire        axi_cdc_o_arready;
  wire [31:0] axi_cdc_o_araddr;
  wire [ 3:0] axi_cdc_o_arid;
  wire [ 7:0] axi_cdc_o_arlen;
  wire [ 2:0] axi_cdc_o_arsize;
  wire [ 1:0] axi_cdc_o_arburst;
  wire [ 0:0] axi_cdc_o_arlock;
  wire [ 3:0] axi_cdc_o_arcache;
  wire [ 2:0] axi_cdc_o_arprot;
  wire        axi_cdc_o_rvalid;
  wire        axi_cdc_o_rready;
  wire [31:0] axi_cdc_o_rdata;
  wire [ 3:0] axi_cdc_o_rid;
  wire [ 1:0] axi_cdc_o_rresp;
  wire        axi_cdc_o_rlast;
  wire        axi_mtx_slv0_awvalid;
  wire        axi_mtx_slv0_awready;
  wire [31:0] axi_mtx_slv0_awaddr;
  wire [ 4:0] axi_mtx_slv0_awid;
  wire [ 7:0] axi_mtx_slv0_awlen;
  wire [ 2:0] axi_mtx_slv0_awsize;
  wire [ 1:0] axi_mtx_slv0_awburst;
  wire [ 0:0] axi_mtx_slv0_awlock;
  wire [ 3:0] axi_mtx_slv0_awcache;
  wire [ 2:0] axi_mtx_slv0_awprot;
  wire        axi_mtx_slv0_wvalid;
  wire        axi_mtx_slv0_wready;
  wire [31:0] axi_mtx_slv0_wdata;
  wire [ 3:0] axi_mtx_slv0_wstrb;
  wire        axi_mtx_slv0_wlast;
  wire        axi_mtx_slv0_bvalid;
  wire        axi_mtx_slv0_bready;
  wire [ 4:0] axi_mtx_slv0_bid;
  wire [ 1:0] axi_mtx_slv0_bresp;
  wire        axi_mtx_slv0_arvalid;
  wire        axi_mtx_slv0_arready;
  wire [31:0] axi_mtx_slv0_araddr;
  wire [ 4:0] axi_mtx_slv0_arid;
  wire [ 7:0] axi_mtx_slv0_arlen;
  wire [ 2:0] axi_mtx_slv0_arsize;
  wire [ 1:0] axi_mtx_slv0_arburst;
  wire [ 0:0] axi_mtx_slv0_arlock;
  wire [ 3:0] axi_mtx_slv0_arcache;
  wire [ 2:0] axi_mtx_slv0_arprot;
  wire        axi_mtx_slv0_rvalid;
  wire        axi_mtx_slv0_rready;
  wire [31:0] axi_mtx_slv0_rdata;
  wire [ 4:0] axi_mtx_slv0_rid;
  wire [ 1:0] axi_mtx_slv0_rresp;
  wire        axi_mtx_slv0_rlast;
  wire        axi_mtx_slv1_awvalid;
  wire        axi_mtx_slv1_awready;
  wire [31:0] axi_mtx_slv1_awaddr;
  wire [ 4:0] axi_mtx_slv1_awid;
  wire [ 7:0] axi_mtx_slv1_awlen;
  wire [ 2:0] axi_mtx_slv1_awsize;
  wire [ 1:0] axi_mtx_slv1_awburst;
  wire [ 0:0] axi_mtx_slv1_awlock;
  wire [ 3:0] axi_mtx_slv1_awcache;
  wire [ 2:0] axi_mtx_slv1_awprot;
  wire        axi_mtx_slv1_wvalid;
  wire        axi_mtx_slv1_wready;
  wire [31:0] axi_mtx_slv1_wdata;
  wire [ 3:0] axi_mtx_slv1_wstrb;
  wire        axi_mtx_slv1_wlast;
  wire        axi_mtx_slv1_bvalid;
  wire        axi_mtx_slv1_bready;
  wire [ 4:0] axi_mtx_slv1_bid;
  wire [ 1:0] axi_mtx_slv1_bresp;
  wire        axi_mtx_slv1_arvalid;
  wire        axi_mtx_slv1_arready;
  wire [31:0] axi_mtx_slv1_araddr;
  wire [ 4:0] axi_mtx_slv1_arid;
  wire [ 7:0] axi_mtx_slv1_arlen;
  wire [ 2:0] axi_mtx_slv1_arsize;
  wire [ 1:0] axi_mtx_slv1_arburst;
  wire [ 0:0] axi_mtx_slv1_arlock;
  wire [ 3:0] axi_mtx_slv1_arcache;
  wire [ 2:0] axi_mtx_slv1_arprot;
  wire        axi_mtx_slv1_rvalid;
  wire        axi_mtx_slv1_rready;
  wire [31:0] axi_mtx_slv1_rdata;
  wire [ 4:0] axi_mtx_slv1_rid;
  wire [ 1:0] axi_mtx_slv1_rresp;
  wire        axi_mtx_slv1_rlast;
  wire        axi_mtx_slv2_awvalid;
  wire        axi_mtx_slv2_awready;
  wire [31:0] axi_mtx_slv2_awaddr;
  wire [ 4:0] axi_mtx_slv2_awid;
  wire [ 7:0] axi_mtx_slv2_awlen;
  wire [ 2:0] axi_mtx_slv2_awsize;
  wire [ 1:0] axi_mtx_slv2_awburst;
  wire [ 0:0] axi_mtx_slv2_awlock;
  wire [ 3:0] axi_mtx_slv2_awcache;
  wire [ 2:0] axi_mtx_slv2_awprot;
  wire        axi_mtx_slv2_wvalid;
  wire        axi_mtx_slv2_wready;
  wire [31:0] axi_mtx_slv2_wdata;
  wire [ 3:0] axi_mtx_slv2_wstrb;
  wire        axi_mtx_slv2_wlast;
  wire        axi_mtx_slv2_bvalid;
  wire        axi_mtx_slv2_bready;
  wire [ 4:0] axi_mtx_slv2_bid;
  wire [ 1:0] axi_mtx_slv2_bresp;
  wire        axi_mtx_slv2_arvalid;
  wire        axi_mtx_slv2_arready;
  wire [31:0] axi_mtx_slv2_araddr;
  wire [ 4:0] axi_mtx_slv2_arid;
  wire [ 7:0] axi_mtx_slv2_arlen;
  wire [ 2:0] axi_mtx_slv2_arsize;
  wire [ 1:0] axi_mtx_slv2_arburst;
  wire [ 0:0] axi_mtx_slv2_arlock;
  wire [ 3:0] axi_mtx_slv2_arcache;
  wire [ 2:0] axi_mtx_slv2_arprot;
  wire        axi_mtx_slv2_rvalid;
  wire        axi_mtx_slv2_rready;
  wire [31:0] axi_mtx_slv2_rdata;
  wire [ 4:0] axi_mtx_slv2_rid;
  wire [ 1:0] axi_mtx_slv2_rresp;
  wire        axi_mtx_slv2_rlast;
  wire        axi_mtx_slv3_awvalid;
  wire        axi_mtx_slv3_awready;
  wire [31:0] axi_mtx_slv3_awaddr;
  wire [ 4:0] axi_mtx_slv3_awid;
  wire [ 7:0] axi_mtx_slv3_awlen;
  wire [ 2:0] axi_mtx_slv3_awsize;
  wire [ 1:0] axi_mtx_slv3_awburst;
  wire [ 0:0] axi_mtx_slv3_awlock;
  wire [ 3:0] axi_mtx_slv3_awcache;
  wire [ 2:0] axi_mtx_slv3_awprot;
  wire        axi_mtx_slv3_wvalid;
  wire        axi_mtx_slv3_wready;
  wire [31:0] axi_mtx_slv3_wdata;
  wire [ 3:0] axi_mtx_slv3_wstrb;
  wire        axi_mtx_slv3_wlast;
  wire        axi_mtx_slv3_bvalid;
  wire        axi_mtx_slv3_bready;
  wire [ 4:0] axi_mtx_slv3_bid;
  wire [ 1:0] axi_mtx_slv3_bresp;
  wire        axi_mtx_slv3_arvalid;
  wire        axi_mtx_slv3_arready;
  wire [31:0] axi_mtx_slv3_araddr;
  wire [ 4:0] axi_mtx_slv3_arid;
  wire [ 7:0] axi_mtx_slv3_arlen;
  wire [ 2:0] axi_mtx_slv3_arsize;
  wire [ 1:0] axi_mtx_slv3_arburst;
  wire [ 0:0] axi_mtx_slv3_arlock;
  wire [ 3:0] axi_mtx_slv3_arcache;
  wire [ 2:0] axi_mtx_slv3_arprot;
  wire        axi_mtx_slv3_rvalid;
  wire        axi_mtx_slv3_rready;
  wire [31:0] axi_mtx_slv3_rdata;
  wire [ 4:0] axi_mtx_slv3_rid;
  wire [ 1:0] axi_mtx_slv3_rresp;
  wire        axi_mtx_slv3_rlast;
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
      .axiIn_awvalid (axi_cdc_i_awvalid),
      .axiIn_awready (axi_cdc_i_awready),
      .axiIn_awaddr  (axi_cdc_i_awaddr),
      .axiIn_awid    (axi_cdc_i_awid),
      .axiIn_awlen   (axi_cdc_i_awlen),
      .axiIn_awsize  (axi_cdc_i_awsize),
      .axiIn_awburst (axi_cdc_i_awburst),
      .axiIn_awlock  (axi_cdc_i_awlock),
      .axiIn_awcache (axi_cdc_i_awcache),
      .axiIn_awprot  (axi_cdc_i_awprot),
      .axiIn_wvalid  (axi_cdc_i_wvalid),
      .axiIn_wready  (axi_cdc_i_wready),
      .axiIn_wdata   (axi_cdc_i_wdata),
      .axiIn_wstrb   (axi_cdc_i_wstrb),
      .axiIn_wlast   (axi_cdc_i_wlast),
      .axiIn_bvalid  (axi_cdc_i_bvalid),
      .axiIn_bready  (axi_cdc_i_bready),
      .axiIn_bid     (axi_cdc_i_bid),
      .axiIn_bresp   (axi_cdc_i_bresp),
      .axiIn_arvalid (axi_cdc_i_arvalid),
      .axiIn_arready (axi_cdc_i_arready),
      .axiIn_araddr  (axi_cdc_i_araddr),
      .axiIn_arid    (axi_cdc_i_arid),
      .axiIn_arlen   (axi_cdc_i_arlen),
      .axiIn_arsize  (axi_cdc_i_arsize),
      .axiIn_arburst (axi_cdc_i_arburst),
      .axiIn_arlock  (axi_cdc_i_arlock),
      .axiIn_arcache (axi_cdc_i_arcache),
      .axiIn_arprot  (axi_cdc_i_arprot),
      .axiIn_rvalid  (axi_cdc_i_rvalid),
      .axiIn_rready  (axi_cdc_i_rready),
      .axiIn_rdata   (axi_cdc_i_rdata),
      .axiIn_rid     (axi_cdc_i_rid),
      .axiIn_rresp   (axi_cdc_i_rresp),
      .axiIn_rlast   (axi_cdc_i_rlast),
      .axiOut_awvalid(axi_cdc_o_awvalid),
      .axiOut_awready(axi_cdc_o_awready),
      .axiOut_awaddr (axi_cdc_o_awaddr),
      .axiOut_awid   (axi_cdc_o_awid),
      .axiOut_awlen  (axi_cdc_o_awlen),
      .axiOut_awsize (axi_cdc_o_awsize),
      .axiOut_awburst(axi_cdc_o_awburst),
      .axiOut_awlock (axi_cdc_o_awlock),
      .axiOut_awcache(axi_cdc_o_awcache),
      .axiOut_awprot (axi_cdc_o_awprot),
      .axiOut_wvalid (axi_cdc_o_wvalid),
      .axiOut_wready (axi_cdc_o_wready),
      .axiOut_wdata  (axi_cdc_o_wdata),
      .axiOut_wstrb  (axi_cdc_o_wstrb),
      .axiOut_wlast  (axi_cdc_o_wlast),
      .axiOut_bvalid (axi_cdc_o_bvalid),
      .axiOut_bready (axi_cdc_o_bready),
      .axiOut_bid    (axi_cdc_o_bid),
      .axiOut_bresp  (axi_cdc_o_bresp),
      .axiOut_arvalid(axi_cdc_o_arvalid),
      .axiOut_arready(axi_cdc_o_arready),
      .axiOut_araddr (axi_cdc_o_araddr),
      .axiOut_arid   (axi_cdc_o_arid),
      .axiOut_arlen  (axi_cdc_o_arlen),
      .axiOut_arsize (axi_cdc_o_arsize),
      .axiOut_arburst(axi_cdc_o_arburst),
      .axiOut_arlock (axi_cdc_o_arlock),
      .axiOut_arcache(axi_cdc_o_arcache),
      .axiOut_arprot (axi_cdc_o_arprot),
      .axiOut_rvalid (axi_cdc_o_rvalid),
      .axiOut_rready (axi_cdc_o_rready),
      .axiOut_rdata  (axi_cdc_o_rdata),
      .axiOut_rid    (axi_cdc_o_rid),
      .axiOut_rresp  (axi_cdc_o_rresp),
      .axiOut_rlast  (axi_cdc_o_rlast)
  );
  AxiCrossbar_1x4 u_AxiCrossbar_1x4 (
      .axiIn_awvalid   (axi_cdc_o_awvalid),
      .axiIn_awready   (axi_cdc_o_awready),
      .axiIn_awaddr    (axi_cdc_o_awaddr),
      .axiIn_awid      (axi_cdc_o_awid),
      .axiIn_awlen     (axi_cdc_o_awlen),
      .axiIn_awsize    (axi_cdc_o_awsize),
      .axiIn_awburst   (axi_cdc_o_awburst),
      .axiIn_awlock    (axi_cdc_o_awlock),
      .axiIn_awcache   (axi_cdc_o_awcache),
      .axiIn_awprot    (axi_cdc_o_awprot),
      .axiIn_wvalid    (axi_cdc_o_wvalid),
      .axiIn_wready    (axi_cdc_o_wready),
      .axiIn_wdata     (axi_cdc_o_wdata),
      .axiIn_wstrb     (axi_cdc_o_wstrb),
      .axiIn_wlast     (axi_cdc_o_wlast),
      .axiIn_bvalid    (axi_cdc_o_bvalid),
      .axiIn_bready    (axi_cdc_o_bready),
      .axiIn_bid       (axi_cdc_o_bid),
      .axiIn_bresp     (axi_cdc_o_bresp),
      .axiIn_arvalid   (axi_cdc_o_arvalid),
      .axiIn_arready   (axi_cdc_o_arready),
      .axiIn_araddr    (axi_cdc_o_araddr),
      .axiIn_arid      (axi_cdc_o_arid),
      .axiIn_arlen     (axi_cdc_o_arlen),
      .axiIn_arsize    (axi_cdc_o_arsize),
      .axiIn_arburst   (axi_cdc_o_arburst),
      .axiIn_arlock    (axi_cdc_o_arlock),
      .axiIn_arcache   (axi_cdc_o_arcache),
      .axiIn_arprot    (axi_cdc_o_arprot),
      .axiIn_rvalid    (axi_cdc_o_rvalid),
      .axiIn_rready    (axi_cdc_o_rready),
      .axiIn_rdata     (axi_cdc_o_rdata),
      .axiIn_rid       (axi_cdc_o_rid),
      .axiIn_rresp     (axi_cdc_o_rresp),
      .axiIn_rlast     (axi_cdc_o_rlast),
      .axiOut_0_awvalid(axi_mtx_slv0_awvalid),
      .axiOut_0_awready(axi_mtx_slv0_awready),
      .axiOut_0_awaddr (axi_mtx_slv0_awaddr),
      .axiOut_0_awid   (axi_mtx_slv0_awid),
      .axiOut_0_awlen  (axi_mtx_slv0_awlen),
      .axiOut_0_awsize (axi_mtx_slv0_awsize),
      .axiOut_0_awburst(axi_mtx_slv0_awburst),
      .axiOut_0_awlock (axi_mtx_slv0_awlock),
      .axiOut_0_awcache(axi_mtx_slv0_awcache),
      .axiOut_0_awprot (axi_mtx_slv0_awprot),
      .axiOut_0_wvalid (axi_mtx_slv0_wvalid),
      .axiOut_0_wready (axi_mtx_slv0_wready),
      .axiOut_0_wdata  (axi_mtx_slv0_wdata),
      .axiOut_0_wstrb  (axi_mtx_slv0_wstrb),
      .axiOut_0_wlast  (axi_mtx_slv0_wlast),
      .axiOut_0_bvalid (axi_mtx_slv0_bvalid),
      .axiOut_0_bready (axi_mtx_slv0_bready),
      .axiOut_0_bid    (axi_mtx_slv0_bid),
      .axiOut_0_bresp  (axi_mtx_slv0_bresp),
      .axiOut_0_arvalid(axi_mtx_slv0_arvalid),
      .axiOut_0_arready(axi_mtx_slv0_arready),
      .axiOut_0_araddr (axi_mtx_slv0_araddr),
      .axiOut_0_arid   (axi_mtx_slv0_arid),
      .axiOut_0_arlen  (axi_mtx_slv0_arlen),
      .axiOut_0_arsize (axi_mtx_slv0_arsize),
      .axiOut_0_arburst(axi_mtx_slv0_arburst),
      .axiOut_0_arlock (axi_mtx_slv0_arlock),
      .axiOut_0_arcache(axi_mtx_slv0_arcache),
      .axiOut_0_arprot (axi_mtx_slv0_arprot),
      .axiOut_0_rvalid (axi_mtx_slv0_rvalid),
      .axiOut_0_rready (axi_mtx_slv0_rready),
      .axiOut_0_rdata  (axi_mtx_slv0_rdata),
      .axiOut_0_rid    (axi_mtx_slv0_rid),
      .axiOut_0_rresp  (axi_mtx_slv0_rresp),
      .axiOut_0_rlast  (axi_mtx_slv0_rlast),
      .axiOut_1_awvalid(axi_mtx_slv1_awvalid),
      .axiOut_1_awready(axi_mtx_slv1_awready),
      .axiOut_1_awaddr (axi_mtx_slv1_awaddr),
      .axiOut_1_awid   (axi_mtx_slv1_awid),
      .axiOut_1_awlen  (axi_mtx_slv1_awlen),
      .axiOut_1_awsize (axi_mtx_slv1_awsize),
      .axiOut_1_awburst(axi_mtx_slv1_awburst),
      .axiOut_1_awlock (axi_mtx_slv1_awlock),
      .axiOut_1_awcache(axi_mtx_slv1_awcache),
      .axiOut_1_awprot (axi_mtx_slv1_awprot),
      .axiOut_1_wvalid (axi_mtx_slv1_wvalid),
      .axiOut_1_wready (axi_mtx_slv1_wready),
      .axiOut_1_wdata  (axi_mtx_slv1_wdata),
      .axiOut_1_wstrb  (axi_mtx_slv1_wstrb),
      .axiOut_1_wlast  (axi_mtx_slv1_wlast),
      .axiOut_1_bvalid (axi_mtx_slv1_bvalid),
      .axiOut_1_bready (axi_mtx_slv1_bready),
      .axiOut_1_bid    (axi_mtx_slv1_bid),
      .axiOut_1_bresp  (axi_mtx_slv1_bresp),
      .axiOut_1_arvalid(axi_mtx_slv1_arvalid),
      .axiOut_1_arready(axi_mtx_slv1_arready),
      .axiOut_1_araddr (axi_mtx_slv1_araddr),
      .axiOut_1_arid   (axi_mtx_slv1_arid),
      .axiOut_1_arlen  (axi_mtx_slv1_arlen),
      .axiOut_1_arsize (axi_mtx_slv1_arsize),
      .axiOut_1_arburst(axi_mtx_slv1_arburst),
      .axiOut_1_arlock (axi_mtx_slv1_arlock),
      .axiOut_1_arcache(axi_mtx_slv1_arcache),
      .axiOut_1_arprot (axi_mtx_slv1_arprot),
      .axiOut_1_rvalid (axi_mtx_slv1_rvalid),
      .axiOut_1_rready (axi_mtx_slv1_rready),
      .axiOut_1_rdata  (axi_mtx_slv1_rdata),
      .axiOut_1_rid    (axi_mtx_slv1_rid),
      .axiOut_1_rresp  (axi_mtx_slv1_rresp),
      .axiOut_1_rlast  (axi_mtx_slv1_rlast),
      .axiOut_2_awvalid(axi_mtx_slv2_awvalid),
      .axiOut_2_awready(axi_mtx_slv2_awready),
      .axiOut_2_awaddr (axi_mtx_slv2_awaddr),
      .axiOut_2_awid   (axi_mtx_slv2_awid),
      .axiOut_2_awlen  (axi_mtx_slv2_awlen),
      .axiOut_2_awsize (axi_mtx_slv2_awsize),
      .axiOut_2_awburst(axi_mtx_slv2_awburst),
      .axiOut_2_awlock (axi_mtx_slv2_awlock),
      .axiOut_2_awcache(axi_mtx_slv2_awcache),
      .axiOut_2_awprot (axi_mtx_slv2_awprot),
      .axiOut_2_wvalid (axi_mtx_slv2_wvalid),
      .axiOut_2_wready (axi_mtx_slv2_wready),
      .axiOut_2_wdata  (axi_mtx_slv2_wdata),
      .axiOut_2_wstrb  (axi_mtx_slv2_wstrb),
      .axiOut_2_wlast  (axi_mtx_slv2_wlast),
      .axiOut_2_bvalid (axi_mtx_slv2_bvalid),
      .axiOut_2_bready (axi_mtx_slv2_bready),
      .axiOut_2_bid    (axi_mtx_slv2_bid),
      .axiOut_2_bresp  (axi_mtx_slv2_bresp),
      .axiOut_2_arvalid(axi_mtx_slv2_arvalid),
      .axiOut_2_arready(axi_mtx_slv2_arready),
      .axiOut_2_araddr (axi_mtx_slv2_araddr),
      .axiOut_2_arid   (axi_mtx_slv2_arid),
      .axiOut_2_arlen  (axi_mtx_slv2_arlen),
      .axiOut_2_arsize (axi_mtx_slv2_arsize),
      .axiOut_2_arburst(axi_mtx_slv2_arburst),
      .axiOut_2_arlock (axi_mtx_slv2_arlock),
      .axiOut_2_arcache(axi_mtx_slv2_arcache),
      .axiOut_2_arprot (axi_mtx_slv2_arprot),
      .axiOut_2_rvalid (axi_mtx_slv2_rvalid),
      .axiOut_2_rready (axi_mtx_slv2_rready),
      .axiOut_2_rdata  (axi_mtx_slv2_rdata),
      .axiOut_2_rid    (axi_mtx_slv2_rid),
      .axiOut_2_rresp  (axi_mtx_slv2_rresp),
      .axiOut_2_rlast  (axi_mtx_slv2_rlast),
      .axiOut_3_awvalid(axi_mtx_slv3_awvalid),
      .axiOut_3_awready(axi_mtx_slv3_awready),
      .axiOut_3_awaddr (axi_mtx_slv3_awaddr),
      .axiOut_3_awid   (axi_mtx_slv3_awid),
      .axiOut_3_awlen  (axi_mtx_slv3_awlen),
      .axiOut_3_awsize (axi_mtx_slv3_awsize),
      .axiOut_3_awburst(axi_mtx_slv3_awburst),
      .axiOut_3_awlock (axi_mtx_slv3_awlock),
      .axiOut_3_awcache(axi_mtx_slv3_awcache),
      .axiOut_3_awprot (axi_mtx_slv3_awprot),
      .axiOut_3_wvalid (axi_mtx_slv3_wvalid),
      .axiOut_3_wready (axi_mtx_slv3_wready),
      .axiOut_3_wdata  (axi_mtx_slv3_wdata),
      .axiOut_3_wstrb  (axi_mtx_slv3_wstrb),
      .axiOut_3_wlast  (axi_mtx_slv3_wlast),
      .axiOut_3_bvalid (axi_mtx_slv3_bvalid),
      .axiOut_3_bready (axi_mtx_slv3_bready),
      .axiOut_3_bid    (axi_mtx_slv3_bid),
      .axiOut_3_bresp  (axi_mtx_slv3_bresp),
      .axiOut_3_arvalid(axi_mtx_slv3_arvalid),
      .axiOut_3_arready(axi_mtx_slv3_arready),
      .axiOut_3_araddr (axi_mtx_slv3_araddr),
      .axiOut_3_arid   (axi_mtx_slv3_arid),
      .axiOut_3_arlen  (axi_mtx_slv3_arlen),
      .axiOut_3_arsize (axi_mtx_slv3_arsize),
      .axiOut_3_arburst(axi_mtx_slv3_arburst),
      .axiOut_3_arlock (axi_mtx_slv3_arlock),
      .axiOut_3_arcache(axi_mtx_slv3_arcache),
      .axiOut_3_arprot (axi_mtx_slv3_arprot),
      .axiOut_3_rvalid (axi_mtx_slv3_rvalid),
      .axiOut_3_rready (axi_mtx_slv3_rready),
      .axiOut_3_rdata  (axi_mtx_slv3_rdata),
      .axiOut_3_rid    (axi_mtx_slv3_rid),
      .axiOut_3_rresp  (axi_mtx_slv3_rresp),
      .axiOut_3_rlast  (axi_mtx_slv3_rlast),
      .clk             (sys_clk),
      .resetn          (sys_resetn)
  );


  //--------------------------------------------------------------------------------
  // MASTER0, CPU
  //--------------------------------------------------------------------------------
  core_top #(
      .TLBNUM(32)
  ) u_core_top (
      .aclk              (cpu_clk),
      .aresetn           (cpu_resetn),
      .intrpt            (intrpt),
      .arid              (axi_cdc_i_arid),
      .araddr            (axi_cdc_i_araddr),
      .arlen             (axi_cdc_i_arlen),
      .arsize            (axi_cdc_i_arsize),
      .arburst           (axi_cdc_i_arburst),
      .arlock            (axi_cdc_i_arlock),
      .arcache           (axi_cdc_i_arcache),
      .arprot            (axi_cdc_i_arprot),
      .arvalid           (axi_cdc_i_arvalid),
      .arready           (axi_cdc_i_arready),
      .rid               (axi_cdc_i_rid),
      .rdata             (axi_cdc_i_rdata),
      .rresp             (axi_cdc_i_rresp),
      .rlast             (axi_cdc_i_rlast),
      .rvalid            (axi_cdc_i_rvalid),
      .rready            (axi_cdc_i_rready),
      .awid              (axi_cdc_i_awid),
      .awaddr            (axi_cdc_i_awaddr),
      .awlen             (axi_cdc_i_awlen),
      .awsize            (axi_cdc_i_awsize),
      .awburst           (axi_cdc_i_awburst),
      .awlock            (axi_cdc_i_awlock),
      .awcache           (axi_cdc_i_awcache),
      .awprot            (axi_cdc_i_awprot),
      .awvalid           (axi_cdc_i_awvalid),
      .awready           (axi_cdc_i_awready),
      .wid               (axi_cdc_i_wid),
      .wdata             (axi_cdc_i_wdata),
      .wstrb             (axi_cdc_i_wstrb),
      .wlast             (axi_cdc_i_wlast),
      .wvalid            (axi_cdc_i_wvalid),
      .wready            (axi_cdc_i_wready),
      .bid               (axi_cdc_i_bid),
      .bresp             (axi_cdc_i_bresp),
      .bvalid            (axi_cdc_i_bvalid),
      .bready            (axi_cdc_i_bready),
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
      .axi_arlock   (axi_mtx_slv0_arlock),
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
      .axi_awlock   (axi_mtx_slv0_awlock),
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
  // SLAVE2, DUMMY
  //--------------------------------------------------------------------------------
  assign axi_mtx_slv2_arready = 1'b1;
  assign axi_mtx_slv2_rid = 5'b0;
  assign axi_mtx_slv2_rdata = 32'b0;
  assign axi_mtx_slv2_rresp = 2'b0;
  assign axi_mtx_slv2_rlast = 1'b0;
  assign axi_mtx_slv2_rvalid = 1'b0;
  assign axi_mtx_slv2_awready = 1'b1;
  assign axi_mtx_slv2_wready = 1'b1;
  assign axi_mtx_slv2_bid = 5'b0;
  assign axi_mtx_slv2_bresp = 2'b0;
  assign axi_mtx_slv2_bvalid = 1'b0;


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


endmodule

