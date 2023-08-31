//top


module top
(
    input                   clk50mhz_i,
    input                   clk100mhz_i,
    // DDR3 SDRAM inter face--------------------------------------
    output  wire            ddr3_reset_n,
    output  wire    [0:0]   ddr3_cke,
    output  wire    [0:0]   ddr3_ck_p, 
    output  wire    [0:0]   ddr3_ck_n,
    output  wire    [0:0]   ddr3_cs_n,
    output  wire            ddr3_ras_n, 
    output  wire            ddr3_cas_n, 
    output  wire            ddr3_we_n,
    output  wire    [2:0]   ddr3_ba,
    output  wire    [13:0]  ddr3_addr,
    output  wire    [0:0]   ddr3_odt,
    output  wire    [1:0]   ddr3_dm,
    inout   wire    [1:0]   ddr3_dqs_p,
    inout   wire    [1:0]   ddr3_dqs_n,
    inout   wire    [15:0]  ddr3_dq,

    // ov5640 interface------------------------------------------ 
    // from ov5640
    input cmos_pclk_i,
    input cmos_href_i,
    input cmos_vsync_i,
    input [7:0] cmos_data_i,
    // i2c interface
    output cmos_sclk_o,
    inout  cmos_sdat_io,
    // to ov5640
    output cmos_xclk_o,

    input read_enable_i,

    // hdmi interface------------------------------------------ 
    // tmds outputs 
    output [3:0]    tmds_out_p_o,
    output [3:0]    tmds_out_n_o  
);



//-----------------------------------------------------------------
// Clocking / Reset
//-----------------------------------------------------------------
wire clk50mhz;
wire clk25mhz;
wire clk125mhz;


artix7_pll_hdmi artix7_pll_hdmi
(

    .clkref_i(clk50mhz_i),      
    .clkout0_o(clk25mhz),    
    .clkout1_o(clk125mhz),
    .clkout2_o()
);



wire clk100mhz;
wire rst_w;
wire clk_ddr_w;
wire clk_ddr_dqs_w;
wire clk_ref_w;
wire cmos_clk_w;


artix7_pll u_pll
(
     .clkref_i(clk100mhz_i)
    ,.clkout0_o(clk100mhz)          // 100
    ,.clkout1_o(clk_ddr_w)          // 400
    ,.clkout2_o(clk_ref_w)          // 200
    ,.clkout3_o(clk_ddr_dqs_w)      // 400 (phase shifted 90 degrees)
);

reset_gen u_rst
(
     .clk_i(clk100mhz)
    ,.rst_o(rst_w)
);

//-----------------------------------------------------------------
// DDR Core + PHY
//-----------------------------------------------------------------
wire [ 13:0]   dfi_address_w;
wire [  2:0]   dfi_bank_w;
wire           dfi_cas_n_w;
wire           dfi_cke_w;
wire           dfi_cs_n_w;
wire           dfi_odt_w;
wire           dfi_ras_n_w;
wire           dfi_reset_n_w;
wire           dfi_we_n_w;
wire [ 31:0]   dfi_wrdata_w;
wire           dfi_wrdata_en_w;
wire [  3:0]   dfi_wrdata_mask_w;
wire           dfi_rddata_en_w;
wire [ 31:0]   dfi_rddata_w;
wire           dfi_rddata_valid_w;
wire [  1:0]   dfi_rddata_dnv_w;

// axi interface
//writre address channel   11
wire           axi4_awready_w;
wire           axi4_awvalid_w;
wire  [  3:0]  axi4_awid_w;
wire  [  1:0]  axi4_awburst_w;
wire  [  7:0]  axi4_awlen_w;
wire  [ 31:0]  axi4_awaddr_w;
wire  [2:0]    axi4_awsize_w;
wire  [2:0]    axi4_awprot_w;
// wire  [1:0]    axi4_awqos_w;        // Burst Type
wire  [1:0]    axi4_awlock_w;       // Lock Type
wire  [3:0]    axi4_awcache_w;      // Cache Type

//write data channel 5
wire           axi4_wvalid_w;
wire           axi4_wready_w;
wire  [ 31:0]  axi4_wdata_w;
wire  [  3:0]  axi4_wstrb_w;
wire           axi4_wlast_w;

//write response channel 4
wire  [  1:0]  axi4_bresp_w;
wire  [  3:0]  axi4_bid_w;
wire           axi4_bvalid_w;
wire           axi4_bready_w;

//read address channel 11
wire           axi4_arvalid_w;
wire           axi4_arready_w;
wire  [  7:0]  axi4_arlen_w;
wire  [ 31:0]  axi4_araddr_w;
wire  [  1:0]  axi4_arburst_w;
wire  [  3:0]  axi4_arid_w;
wire  [  3:0]  axi4_arcache_w;
wire  [  2:0]  axi4_arprot_w;
//wire  [  3:0]  axi4_arqos_w;
wire  [  1:0]  axi4_arlock_w;
wire  [  2:0]  axi4_arsize_w;

//read data channel 6
wire           axi4_rvalid_w;
wire           axi4_rready_w;
wire  [  3:0]  axi4_rid_w;
wire  [  1:0]  axi4_rresp_w;
wire  [ 31:0]  axi4_rdata_w;
wire           axi4_rlast_w;

// wire test_fail;
// to ddr_user_control
wire hs_w;
wire vs_w;
wire[15:0] rgb_o_w;
wire[31:0] rgb_w;
assign  rgb_w[31:0] = {16'b0,rgb_o_w};
wire clk_ce_w;

wire blank_hdmi_w;
wire [31:0] data_rd_rgb_w;




ddr3_axi #(
     .DDR_WRITE_LATENCY(4)
    ,.DDR_READ_LATENCY(4)
    ,.DDR_MHZ(100)
)
u_ddr
(
    // Inputs
     .clk_i(clk100mhz)
    ,.rst_i(rst_w)
    ,.inport_awvalid_i(axi4_awvalid_w)
    ,.inport_awaddr_i(axi4_awaddr_w)
    ,.inport_awid_i(axi4_awid_w)
    ,.inport_awlen_i(axi4_awlen_w)
    ,.inport_awburst_i(axi4_awburst_w)
    ,.inport_wvalid_i(axi4_wvalid_w)
    ,.inport_wdata_i(axi4_wdata_w)
    ,.inport_wstrb_i(axi4_wstrb_w)
    ,.inport_wlast_i(axi4_wlast_w)
    ,.inport_bready_i(axi4_bready_w)
    ,.inport_arvalid_i(axi4_arvalid_w)
    ,.inport_araddr_i(axi4_araddr_w)
    ,.inport_arid_i(axi4_arid_w)
    ,.inport_arlen_i(axi4_arlen_w)
    ,.inport_arburst_i(axi4_arburst_w)
    ,.inport_rready_i(axi4_rready_w)

    ,.dfi_rddata_i(dfi_rddata_w)
    ,.dfi_rddata_valid_i(dfi_rddata_valid_w)
    ,.dfi_rddata_dnv_i(dfi_rddata_dnv_w)

    // Outputs
    ,.inport_awready_o(axi4_awready_w)
    ,.inport_wready_o(axi4_wready_w)
    ,.inport_bvalid_o(axi4_bvalid_w)
    ,.inport_bresp_o(axi4_bresp_w)
    ,.inport_bid_o(axi4_bid_w)
    ,.inport_arready_o(axi4_arready_w)
    ,.inport_rvalid_o(axi4_rvalid_w)
    ,.inport_rdata_o(axi4_rdata_w)
    ,.inport_rresp_o(axi4_rresp_w)
    ,.inport_rid_o(axi4_rid_w)
    ,.inport_rlast_o(axi4_rlast_w)

    ,.dfi_address_o(dfi_address_w)
    ,.dfi_bank_o(dfi_bank_w)
    ,.dfi_cas_n_o(dfi_cas_n_w)
    ,.dfi_cke_o(dfi_cke_w)
    ,.dfi_cs_n_o(dfi_cs_n_w)
    ,.dfi_odt_o(dfi_odt_w)
    ,.dfi_ras_n_o(dfi_ras_n_w)
    ,.dfi_reset_n_o(dfi_reset_n_w)
    ,.dfi_we_n_o(dfi_we_n_w)
    ,.dfi_wrdata_o(dfi_wrdata_w)
    ,.dfi_wrdata_en_o(dfi_wrdata_en_w)
    ,.dfi_wrdata_mask_o(dfi_wrdata_mask_w)
    ,.dfi_rddata_en_o(dfi_rddata_en_w)
);


ddr3_dfi_phy #(
     .DQS_TAP_DELAY_INIT(27)
    ,.DQ_TAP_DELAY_INIT(0)
    ,.TPHY_RDLAT(5)
)
u_phy
(
     .clk_i(clk100mhz)
    ,.rst_i(rst_w)

    ,.clk_ddr_i(clk_ddr_w)
    ,.clk_ddr90_i(clk_ddr_dqs_w)
    ,.clk_ref_i(clk_ref_w)

    ,.cfg_valid_i(1'b0)
    ,.cfg_i(32'b0)

    ,.dfi_address_i(dfi_address_w)
    ,.dfi_bank_i(dfi_bank_w)
    ,.dfi_cas_n_i(dfi_cas_n_w)
    ,.dfi_cke_i(dfi_cke_w)
    ,.dfi_cs_n_i(dfi_cs_n_w)
    ,.dfi_odt_i(dfi_odt_w)
    ,.dfi_ras_n_i(dfi_ras_n_w)
    ,.dfi_reset_n_i(dfi_reset_n_w)
    ,.dfi_we_n_i(dfi_we_n_w)
    ,.dfi_wrdata_i(dfi_wrdata_w)
    ,.dfi_wrdata_en_i(dfi_wrdata_en_w)
    ,.dfi_wrdata_mask_i(dfi_wrdata_mask_w)
    ,.dfi_rddata_en_i(dfi_rddata_en_w)
    ,.dfi_rddata_o(dfi_rddata_w)
    ,.dfi_rddata_valid_o(dfi_rddata_valid_w)
    ,.dfi_rddata_dnv_o(dfi_rddata_dnv_w)
    
    // ddr3 interface
    ,.ddr3_ck_p_o(ddr3_ck_p)
    ,.ddr3_ck_n_o(ddr3_ck_n)
    ,.ddr3_cke_o(ddr3_cke)
    ,.ddr3_reset_n_o(ddr3_reset_n)
    ,.ddr3_ras_n_o(ddr3_ras_n)
    ,.ddr3_cas_n_o(ddr3_cas_n)
    ,.ddr3_we_n_o(ddr3_we_n)
    ,.ddr3_cs_n_o(ddr3_cs_n)
    ,.ddr3_ba_o(ddr3_ba)
    ,.ddr3_addr_o(ddr3_addr[13:0])
    ,.ddr3_odt_o(ddr3_odt)
    ,.ddr3_dm_o(ddr3_dm)
    ,.ddr3_dq_io(ddr3_dq)
    ,.ddr3_dqs_p_io(ddr3_dqs_p)
    ,.ddr3_dqs_n_io(ddr3_dqs_n)    
);

ddr_user_ctrl  ddr_user_ctrl_inst(

    .ui_clk(clk100mhz) ,
    .ui_rst(!rst_w) ,
    .pingpang(pingpang) ,
    
    // user interface
    // write input ------------------------------------------
    .wr_b_addr(32'd0) , //写 DDR 首地址
    .wr_e_addr() , //写 DDR 末地址

    .user_wr_clk(clk25mhz), //写 FIFO 写时钟
    .data_wren(clk_ce_w) , //写 FIFO 写请求
    .data_wr(rgb_w) ,
    .wr_rst(1'b0) ,


    // read input ------------------------------------------
    .rd_b_addr(32'd0) , //读 DDR 首地址
    .rd_e_addr() , //读 DDR 末地址

    .user_rd_clk(clk25mhz), //读 FIFO 读时钟
    .data_rden(blank_hdmi_w) , //读 FIFO 读请求 
    .data_rd(data_rd_rgb_w) , 
    .rd_rst(1'b0) ,
    .read_enable(read_enable_i),
    
    //ddr3 interface------------------------------------------
    .inport_awvalid_o(axi4_awvalid_w),
    .inport_awaddr_o(axi4_awaddr_w),
    .inport_awid_o(axi4_awid_w),
    .inport_awlen_o(axi4_awlen_w),
    .inport_awburst_o(axi4_awburst_w),
    .inport_wvalid_o(axi4_wvalid_w),
    .inport_wdata_o(axi4_wdata_w),
    .inport_wstrb_o(axi4_wstrb_w),
    .inport_wlast_o(axi4_wlast_w),
    .inport_bready_o(axi4_bready_w),
    .inport_arvalid_o(axi4_arvalid_w),
    .inport_araddr_o(axi4_araddr_w),
    .inport_arid_o(axi4_arid_w),
    .inport_arlen_o(axi4_arlen_w),
    .inport_arburst_o(axi4_arburst_w),
    .inport_rready_o(axi4_rready_w),

    .inport_awready_i(axi4_awready_w),
    .inport_wready_i(axi4_wready_w),
    .inport_bvalid_i(axi4_bvalid_w),
    .inport_bresp_i(axi4_bresp_w),
    .inport_bid_i(axi4_bid_w),
    .inport_arready_i(axi4_arready_w),
    .inport_rvalid_i(axi4_rvalid_w),
    .inport_rdata_i(axi4_rdata_w),
    .inport_rresp_i(axi4_rresp_w),
    .inport_rid_i(axi4_rid_w),
    .inport_rlast_i(axi4_rlast_w)
);


OV5640_top OV5640_top_inst(
        .clk_i(clk25mhz),            
        // clk and rst 
        .cmos_clk_i(clk25mhz),
        .rst_i(!rst_w),

        // from ov5640
        .cmos_pclk_i(cmos_pclk_i),
        .cmos_href_i(cmos_href_i),
        .cmos_vsync_i(cmos_vsync_i),
        .cmos_data_i(cmos_data_i),

        // i2c interface
        .cmos_sclk_o(cmos_sclk_o),
        .cmos_sdat_io(cmos_sdat_io),
        // to ov5640  
        .cmos_xclk_o(cmos_xclk_o),         

        // to ddr_top
        .hs_o(hs_w),
        .vs_o(vs_w),
        .rgb_o(rgb_o_w),
        .clk_ce_o(clk_ce_w)
);


hdmi_top hdmi_top_inst(
        // clk and rst Inputs
        .clk_i(clk25mhz),
        .rst_i(!rst_w),
        .clk_x5_i(clk125mhz),

        // rgb inputs 8/8/8 bits data_rd_rgb_w
        .vga_red_i(data_rd_rgb_w[7:0]),
        .vga_green_i(data_rd_rgb_w[15:8]),
        .vga_blue_i(data_rd_rgb_w[23:16]),

        // tmds outputs 
        .blank_hdmi_o(blank_hdmi_w),
        .tmds_out_p(tmds_out_p_o),
        .tmds_out_n(tmds_out_n_o)  

);
endmodule