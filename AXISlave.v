`timescale 10ns/1ns
`DEFINE Data_width 32
//Top Level Module
module AXISlave();
	//Input Output Parameters
	input					ACLK, ARESET;
	input	wire	[31:0]	 		AWADDR;
	input	wire	[31:0]			ARADDR; 
	input	wire	[3:0]			AWLEN;
	input	wire	[3:0]			ARLEN;
	input	wire	[2:0]			AWSIZE; 
	input	wire	[2:0]			ARSIZE; 
	input	wire	[1:0]			AWBURST; 
	input	wire	[1:0]			ARBURST; 
	input	wire				AWVALID; 
	input	wire				ARVALID; 
	output	reg				AWREADY; 
	output	reg				ARREADY; 
	input	wire	[`Data_width-1:0]	WDATA;
	output	reg	[`Data_width-1:0]	RDATA;
	input	wire	[3:0]			WSTRB;
	input	wire				WLAST;
	output	reg				RLAST;
	input	wire				WVALID; 
	output	reg				RVALID; 
	output	reg				WREADY; 
	input	wire				RREADY; 
	output	reg				BID;
	output	reg				BREADY; 
	input	wire				BVALID;
	//LOCAL VARIABLES
	reg 	[1:0]			psr, nsr, psw, nsw;

	reg 	[31:0]			START_ADDR, ALIGNED_ADDR, START_ADDR_W, ALIGNED_ADDR_W;
	integer				NUMBER_BYTES, NUMBER_BYTES_W;
	reg	[3:0]			BURST_LEN, BURST_LEN_W;
	reg				ALIGNED, ALIGNED_W;
	reg	[6:0]			DT_SIZE, DT_SIZE_W;
	reg	[1:0]			BURST, BURST_W;
	reg	[31:0]			LOWWRAP, UPWRAP, LOWWRAP_W, UPWRAP_W;
	integer				ubl, lbl, ubl_W, lbl_W;
	//Parameters
	parameter 	FIXED 		= 2'B00,
			INCR  		= 2'B01,
			WRAP  		= 2'B10,
			IDLE  		= 2'B00,
			ACTIVE		= 2'B01,
			DONE		= 2'B11;
	//Memory
	reg [7:0] mem [0:1024];
	//Synchronous State Transition
	always @ (posedge ACLK) begin 
		psr <= nsr;
		psw <= nsw;
	end
	//----------------------------------------------------------------Asynchronous Operations-----------------------------------------------------------------------
	//---------------------------------------------------Seperate Read Channel for Duplex operation-----------------------------------------------------------------
	always @ (psr or nsr or negedge ARESETn or ARVALID) begin
		//Active low Reset
		if(!ARESETn) begin
			ARREADY		= 1;
			RDATA		= 0;
			RVALID		= 0;
			RLAST		= 0;
			nsr		= IDLE;
		end
		else if (ARESETn and ARVALID) begin
			case(psr)
			IDLE:	if(ARVALID) begin
					//Initial constraints
					ARREADY		= 1'b1;
					START_ADDR	= ARADDR;
					NUMBER_BYTES	= 2 ** ARSIZE;
					BURST_LEN	= ARLEN + 1;
					RVALID		= 1'b0;
					ALIGNED_ADDR	= (ARADDR/NUMBER_BYTES) * NUMBER_BYTES;
					ALIGNED		= (ALIGNED_ADDR == START_ADDR);
					DT_SIZE		= NUMBER_BYTES * BURST_LEN;
					BURST		= ARBURST;
					LOWWRAP		= (START_ADDR/DT_SIZE) * DT_SIZE;
					UPWRAP 		= LOWWRAP + DT_SIZE;
					nsr		= ACTIVE;
					if(!ALIGNED) 	START_ADDR	= ALIGNED_ADDR;
				end
			ACTIVE:	if(BURST_LEN > 0 && RVALID) begin
					RVALID 		= 1'B1;
					TEMP_START_ADDR = START_ADDR;
					case(BURST)
					//FIXED BURST 
					FIXED:	begin
						lbl		= START_ADDR - ((START_ADDR/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl 		= lbl + NUMBER_BYTES - 1;
						for(COUNT_R = lbl; COUNT_R <= ubl; COUNT_R = COUNT_R + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end						
						end
					//INCREMENTING BURST
					INCR:	begin
						lbl		= START_ADDR - ((START_ADDR/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl 		= lbl + NUMBER_BYTES - 1;
						for(COUNT_R = lbl; COUNT_R <= ubl; COUNT_R = COUNT_R + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end	
						START_ADDR	= START_ADDR + NUMBER_BYTES;					
						end
					//WRAPPING BURST
					WRAP:	begin
						lbl		= START_ADDR - ((START_ADDR/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl 		= lbl + NUMBER_BYTES - 1;
						for(COUNT_R = lbl; COUNT_R <= ubl; COUNT_R = COUNT_R + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end
						//WRAP CHECK
						if(START_ADDR >= UPWRAP) 
							START_ADDR 		= LOWWRAP;		
						else	START_ADDR		= START_ADDR + NUMBER_BYTES;
						end
					default://RDATA				= 0;
					endcase
					BURST_LEN	= BURST_LEN - 1;
				end
				else begin
					nsr		= DONE;
					RLAST 		= 1'b1;				
				end
			DONE:	begin
					nsr   		= IDLE;
					RLAST		= 1'b0;
					RVALID 		= 1'b0;
				end	
			default:	nsr		= IDLE;
			endcase
		end
	end
	//----------------------------------------------------Seperate Write Channel for Duplex operation-----------------------------------------------------------
	always @ (psw or nsw or negedge ARESETn or AWVALID) begin
		//Active low Reset
		if(!ARESETn) begin
			AWREADY		= 1;
			BVALID		= 0;
		end
		else if (ARESETn and AWVALID) begin
			case(psr)
			IDLE:	if(AWVALID) begin
					//Initial constraints
					AWREADY		= 1'b1;
					START_ADDR_W	= AWADDR;
					NUMBER_BYTES_W	= 2 ** AWSIZE;
					BURST_LEN_W	= AWLEN + 1;
					ALIGNED_ADDR_W	= (AWADDR/NUMBER_BYTES_W) * NUMBER_BYTES_W;
					ALIGNED_W	= (ALIGNED_ADDR_W == START_ADDR_W);
					DT_SIZE_W	= NUMBER_BYTES_W * BURST_LEN_W;
					BURST_W		= AWBURST;
					LOWWRAP_W	= (START_ADDR_W/DT_SIZE_W) * DT_SIZE_W;
					UPWRAP_W 	= LOWWRAP_W + DT_SIZE_W;
					nsr		= ACTIVE;
					if(!ALIGNED_W) 	START_ADDR_W	= ALIGNED_ADDR_W;
				end
			ACTIVE:	if(BURST_LEN_W > 0) begin
					TEMP_START_ADDR_W = START_ADDR_W;
					case(BURST_W)
					//FIXED BURST 
					FIXED:	begin
						lbl_W		= START_ADDR_W - ((START_ADDR_W/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl_W 		= lbl_W + NUMBER_BYTES_W - 1;
						for(COUNT_W = lbl_W; COUNT_W <= ubl_W; COUNT_W = COUNT_W + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end						
						end
					//INCREMENTING BURST
					INCR:	begin
						lbl		= START_ADDR - ((START_ADDR/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl 		= lbl + NUMBER_BYTES - 1;
						for(COUNT_R = lbl; COUNT_R <= ubl; COUNT_R = COUNT_R + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end	
						START_ADDR	= START_ADDR + NUMBER_BYTES;					
						end
					//WRAPPING BURST
					WRAP:	begin
						lbl		= START_ADDR - ((START_ADDR/DATA_BUS_BYTES) * DATA_BUS_BYTES);
						ubl 		= lbl + NUMBER_BYTES - 1;
						for(COUNT_R = lbl; COUNT_R <= ubl; COUNT_R = COUNT_R + 1) begin
							RDATA[8*COUNT_R+:7]	= mem[TEMP_START_ADDR];
							TEMP_START_ADDR 	= TEMP_START_ADDR + 1; 	
						end
						//WRAP CHECK
						if(START_ADDR >= UPWRAP) 
							START_ADDR 		= LOWWRAP;		
						else	START_ADDR		= START_ADDR + NUMBER_BYTES;
						end
					default://RDATA				= 0;
					endcase
					BURST_LEN	= BURST_LEN - 1;
				end
				else begin
					nsr		= DONE;
					RLAST 		= 1'b1;				
				end
			DONE:	begin
					nsr   		= IDLE;
					RLAST		= 1'b0;
					RVALID 		= 1'b0;
				end	
			default:	nsr		= IDLE;
			endcase
		end
	end
endmodule
//Program ends