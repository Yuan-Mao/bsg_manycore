/**
 *  lsu.v
 *
 *  load store unit
 *
 *  this module is responsible of address generation, address checking,
 *  handshaking glue logic, etc.
 *
 */

`include "bsg_defines.sv"

module lsu

  // Import address parameters
  import bsg_manycore_pkg::*;
  import bsg_manycore_addr_pkg::*;
  import bsg_vanilla_pkg::*;

  #(`BSG_INV_PARAM(data_width_p)
    , `BSG_INV_PARAM(pc_width_p)
    , `BSG_INV_PARAM(dmem_size_p)

    , localparam dmem_addr_width_lp=`BSG_SAFE_CLOG2(dmem_size_p)
    , data_mask_width_lp=(data_width_p>>3)
    , reg_addr_width_lp=RV32_reg_addr_width_gp
  )
  (
    input clk_i
    , input reset_i

    // from EXE
    , input decode_s exe_decode_i
    , input [data_width_p-1:0] exe_rs1_i
    , input [data_width_p-1:0] exe_rs2_i
    , input [reg_addr_width_lp-1:0] exe_rd_i
    , input [RV32_Iimm_width_gp-1:0] mem_offset_i
    , input [data_width_p-1:0] pc_plus4_i
    , input icache_miss_i

    // to network TX
    , output remote_req_s remote_req_o
    , output logic remote_req_v_o

    // to MEM
    , output logic dmem_v_o
    , output logic dmem_w_o
    , output logic [dmem_addr_width_lp-1:0] dmem_addr_o
    , output logic [data_width_p-1:0] dmem_data_o
    , output logic [data_mask_width_lp-1:0] dmem_mask_o 

    , output logic reserve_o
    , output logic [1:0] byte_sel_o 

  );



  logic [data_width_p-1:0] mem_addr;
  logic [data_width_p-1:0] miss_addr;

  assign mem_addr = exe_rs1_i + `BSG_SIGN_EXTEND(mem_offset_i, data_width_p);
  assign miss_addr = (pc_plus4_i - 'h4) | bsg_dram_npa_prefix_gp;

  // cbo target address
  wire [data_width_p-1:0] cbo_addr = exe_rs1_i;

  // store data mask
  //
  logic [data_width_p-1:0] store_data;
  logic [data_mask_width_lp-1:0] store_mask;

  always_comb begin
    if (exe_decode_i.is_byte_op) begin
      store_data = {4{exe_rs2_i[7:0]}};
      store_mask = {
         mem_addr[1] &  mem_addr[0],
         mem_addr[1] & ~mem_addr[0],
        ~mem_addr[1] &  mem_addr[0],
        ~mem_addr[1] & ~mem_addr[0]
      };
    end
    else if (exe_decode_i.is_hex_op) begin
      store_data = {2{exe_rs2_i[15:0]}};
      store_mask = {
        {2{mem_addr[1]}},
        {2{~mem_addr[1]}}
      };
    end
    else begin
      // also covers AMO op
      store_data = exe_rs2_i;
      store_mask = 4'b1111;
    end
  end


  // to local DMEM
  //
  wire is_local_dmem_addr = (mem_addr ==? 32'b00000000_00000000_0000????_????????);

  assign dmem_v_o = is_local_dmem_addr &
    (exe_decode_i.is_load_op | exe_decode_i.is_store_op |
     exe_decode_i.is_lr_op | exe_decode_i.is_lr_aq_op);
  assign dmem_w_o = exe_decode_i.is_store_op;
  assign dmem_addr_o = mem_addr[2+:dmem_addr_width_lp]; 
  assign dmem_data_o = store_data;
  assign dmem_mask_o = store_mask;

  assign byte_sel_o = mem_addr[1:0];

  // CBO
  wire is_cbo_clean = exe_decode_i.is_cbo_clean;
  wire is_cbo_flush = exe_decode_i.is_cbo_flush;
  wire is_cbo_inval = exe_decode_i.is_cbo_inval;
  wire is_custom_cbo_taglv = exe_decode_i.is_custom_cbo_taglv;
  wire is_custom_cbo_tagla = exe_decode_i.is_custom_cbo_tagla;
  wire is_custom_cbo_tagfl = exe_decode_i.is_custom_cbo_tagfl;
  wire is_cbo = exe_decode_i.is_cbo_clean |
    exe_decode_i.is_cbo_flush | exe_decode_i.is_cbo_inval;
  wire is_custom_cbo = is_custom_cbo_taglv | is_custom_cbo_tagla | is_custom_cbo_tagfl;
  // remote request
  // 1) icache fetch
  // 2) remote store
  // 3) remote load
  // 4) atomic
  bsg_manycore_load_info_s load_info;
  

  always_comb begin
    // load info
    if (icache_miss_i) begin
      load_info = '{
        float_wb: 1'b0,
        icache_fetch: 1'b1,
        is_unsigned_op: 1'b0,
        is_byte_op: 1'b0,
        is_hex_op: 1'b0,
        part_sel: 2'b00
      };
    end
    else begin
      load_info = '{
        float_wb: exe_decode_i.write_frd,
        icache_fetch: 1'b0,
        is_unsigned_op: exe_decode_i.is_load_unsigned,
        is_byte_op: exe_decode_i.is_byte_op,
        is_hex_op: exe_decode_i.is_hex_op,
        part_sel: mem_addr[1:0]
      };
    end


    remote_req_o = '{
      access_type : ((is_cbo | is_custom_cbo)
        ? e_vanilla_cbo
        : exe_decode_i.is_store_op
          ? e_vanilla_write
          : e_vanilla_read),
      is_amo_op : exe_decode_i.is_amo_op, 
      amo_type : exe_decode_i.amo_type,
      mask: store_mask,
      load_info : load_info,
      reg_id : exe_rd_i,
      // TODO: make it cleaner
      cache_op : (is_cbo_clean
        ? e_afl
        : is_cbo_flush
          ? e_aflinv
          : is_cbo_inval
            ? e_ainv
            : is_custom_cbo_taglv
              ? e_taglv
              : is_custom_cbo_tagla
                ? e_tagla
                : is_custom_cbo_tagfl
                  ? e_tagfl
                  : e_tagfl), // e_tagfl used as default

      data : store_data,
      addr : (icache_miss_i
        ? miss_addr
        : is_cbo_flush
          ? cbo_addr
          : mem_addr)
    }; 

  end


  assign remote_req_v_o = icache_miss_i |
    ((exe_decode_i.is_load_op | exe_decode_i.is_store_op | exe_decode_i.is_amo_op | is_cbo) & ~is_local_dmem_addr);

  // reserve
  // only valid on local DMEM
  assign reserve_o = exe_decode_i.is_lr_op & is_local_dmem_addr;




  // synopsys translate_off

  always_ff @ (negedge clk_i) begin
    if (~reset_i) begin
      if (exe_decode_i.is_amo_op)
        assert(~is_local_dmem_addr) else $error("[BSG_ERROR] atomic operations cannot be made on local DMEM address space.");

      if (exe_decode_i.is_lr_op | exe_decode_i.is_lr_aq_op)
        assert(is_local_dmem_addr) else $error("[BSG_ERROR] LR operation can only be made on local DMEM address space.");

    end
  end


  // synopsys translate_on



endmodule

`BSG_ABSTRACT_MODULE(lsu)
