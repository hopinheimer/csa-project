#include<iostream>
#include<string>
#include<vector>
#include<bitset>
#include<fstream>
#include<cstdint>

using namespace std;

#define MemSize 1000 // memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.

struct IFStruct {
    bitset<32>  PC;
    bool        nop;  
};

struct IDStruct {
    bitset<32>  Instr;
    bool        nop;
};

struct EXStruct {
    bitset<32>  Read_data1;
    bitset<32>  Read_data2;
    bitset<16>  Imm;
    bitset<5>   Rs;
    bitset<5>   Rt;
    bitset<5>   Wrt_reg_addr;
    bool        is_I_type;
    bool        rd_mem;
    bool        wrt_mem; 
    bool        alu_op;     //1 for addu, lw, sw, 0 for subu 
    bool        wrt_enable;
    bool        nop;  
};

struct MEMStruct {
    bitset<32>  ALUresult;
    bitset<32>  Store_data;
    bitset<5>   Rs;
    bitset<5>   Rt;    
    bitset<5>   Wrt_reg_addr;
    bool        rd_mem;
    bool        wrt_mem; 
    bool        wrt_enable;    
    bool        nop;    
};

struct WBStruct {
    bitset<32>  Wrt_data;
    bitset<5>   Rs;
    bitset<5>   Rt;     
    bitset<5>   Wrt_reg_addr;
    bool        wrt_enable;
    bool        nop;
};

struct stateStruct {
    IFStruct    IF;
    IDStruct    ID;
    EXStruct    EX;
    MEMStruct   MEM;
    WBStruct    WB;
};

class InsMem
{
	public:
		string id, ioDir;
        InsMem(string name, string ioDir) {       
			id = name;
			IMem.resize(MemSize);
            ifstream imem;
			string line;
			int i=0;
			imem.open(ioDir + "\\imem.txt");
			if (imem.is_open())
			{
				while (getline(imem,line))
				{      
					IMem[i] = bitset<8>(line);
					i++;
				}                    
			}
            else cout<<"Unable to open IMEM input file.";
			imem.close();                     
		}

		bitset<32> readInstr(bitset<32> ReadAddress) {    
			// read instruction memory
			// return bitset<32> val
			// Convert the bitset address to an unsigned integer
			unsigned int addr = (unsigned int)(ReadAddress.to_ulong());

			// Check for address overflow
			if (addr + 3 >= MemSize) {
				cout << "Address out of bounds in readInstr" << endl;
				return bitset<32>(0);
			}

			// Read 4 bytes from IMem in Big-Endian order
			bitset<8> byte0 = IMem[addr];
			bitset<8> byte1 = IMem[addr + 1];
			bitset<8> byte2 = IMem[addr + 2];
			bitset<8> byte3 = IMem[addr + 3];

			// Assemble the bytes into a 32-bit instruction
			bitset<32> instruction(0);
			instruction = (bitset<32>(byte0.to_ulong()) << 24) |
				(bitset<32>(byte1.to_ulong()) << 16) |
				(bitset<32>(byte2.to_ulong()) << 8) |
				bitset<32>(byte3.to_ulong());

			return instruction;
		}     
      
    private:
        vector<bitset<8> > IMem;     
};
      
class DataMem    
{
    public: 
		string id, opFilePath, ioDir;
        DataMem(string name, string ioDir) : id{name}, ioDir{ioDir} {
            DMem.resize(MemSize);
			opFilePath = ioDir + "\\" + name + "_DMEMResult.txt";
            ifstream dmem;
            string line;
            int i=0;
            dmem.open(ioDir + "\\dmem.txt");
            if (dmem.is_open())
            {
                while (getline(dmem,line))
                {      
                    DMem[i] = bitset<8>(line);
                    i++;
                }
            }
            else cout<<"Unable to open DMEM input file.";
                dmem.close();          
        }
		
        bitset<32> readDataMem(bitset<32> Address) {	
			// read data memory
			// return bitset<32> val
			unsigned int addr = (unsigned int)(Address.to_ulong());

			// Check for address overflow
			if (addr + 3 >= MemSize) {
				cout << "Address out of bounds in readDataMem" << endl;
				return bitset<32>(0);
			}

			// Read 4 bytes from DMem in Big-Endian order
			bitset<8> byte0 = DMem[addr];
			bitset<8> byte1 = DMem[addr + 1];
			bitset<8> byte2 = DMem[addr + 2];
			bitset<8> byte3 = DMem[addr + 3];

			// Assemble the bytes into a 32-bit data word
			bitset<32> data(0);
			data = (bitset<32>(byte0.to_ulong()) << 24) |
				(bitset<32>(byte1.to_ulong()) << 16) |
				(bitset<32>(byte2.to_ulong()) << 8) |
				bitset<32>(byte3.to_ulong());

			return data;
		}
            
        void writeDataMem(bitset<32> Address, bitset<32> WriteData) {
			// write into memory
			unsigned int addr = (unsigned int)(Address.to_ulong());

			// Check for address overflow
			if (addr + 3 >= MemSize) {
				cout << "Address out of bounds in writeDataMem" << endl;
				return;
			}

			// Split the 32-bit WriteData into 4 bytes
			bitset<8> byte0 = bitset<8>((WriteData >> 24).to_ulong());
			bitset<8> byte1 = bitset<8>((WriteData >> 16).to_ulong());
			bitset<8> byte2 = bitset<8>((WriteData >> 8).to_ulong());
			bitset<8> byte3 = bitset<8>(WriteData.to_ulong());

			// Write the bytes into DMem in Big-Endian order
			DMem[addr] = byte0;
			DMem[addr + 1] = byte1;
			DMem[addr + 2] = byte2;
			DMem[addr + 3] = byte3;
        }   
                     
        void outputDataMem() {
            ofstream dmemout;
            dmemout.open(opFilePath, std::ios_base::trunc);
            if (dmemout.is_open()) {
                for (int j = 0; j< 1000; j++)
                {     
                    dmemout << DMem[j]<<endl;
                }
                     
            }
            else cout<<"Unable to open "<<id<<" DMEM result file." << endl;
            dmemout.close();
        }             

    private:
		vector<bitset<8> > DMem;      
};

class RegisterFile
{
    public:
		string outputFile;
     	RegisterFile(string ioDir): outputFile {ioDir + "RFResult.txt"} {
			Registers.resize(32);  
			Registers[0] = bitset<32> (0);  
        }
	
        bitset<32> readRF(bitset<5> Reg_addr) {   
            // Fill in
			unsigned int addr = Reg_addr.to_ulong();

			// Check if the address is within valid range
			if (addr >= Registers.size()) {
				cout << "Invalid register address in readRF: " << addr << endl;
				return bitset<32>(0);
			}

			// Return the value stored in the register
			return Registers[addr];
        }
    
        void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data) {
            // Fill in
			unsigned int addr = Reg_addr.to_ulong();

			// Register x0 is hardwired to zero and should not be written to
			if (addr == 0) {
				return;
			}

			// Check if the address is within valid range
			if (addr >= Registers.size()) {
				cout << "Invalid register address in writeRF: " << addr << endl;
				return;
			}

			// Write the data to the register
			Registers[addr] = Wrt_reg_data;
        }
		 
		void outputRF(int cycle) {
			ofstream rfout;
			if (cycle == 0)
				rfout.open(outputFile, std::ios_base::trunc);
			else 
				rfout.open(outputFile, std::ios_base::app);
			if (rfout.is_open())
			{
				rfout << "----------------------------------------------------------------------" << endl;
				rfout << "State of RF after executing cycle:" << cycle << endl;
				for (int j = 0; j<32; j++)
				{
					rfout << Registers[j]<<endl;
				}
			}
			else cout<<"Unable to open RF output file."<<endl;
			rfout.close();
		} 

	private:
		vector<bitset<32> >Registers;
};

class Core {
	public:
		RegisterFile myRF;
		uint32_t cycle = 0;
		bool halted = false;
		string ioDir;
		struct stateStruct state, nextState;
		InsMem &ext_imem;
		DataMem &ext_dmem;
		
		Core(string ioDir, InsMem &imem, DataMem &dmem): myRF(ioDir), ioDir{ioDir}, ext_imem {imem}, ext_dmem {dmem} {}

		virtual void step() {}

		virtual void printState() {}
};

class SingleStageCore : public Core {
	public:
		uint32_t instruction_count = 0;

		SingleStageCore(string ioDir, InsMem &imem, DataMem &dmem): Core(ioDir + "\\SS_", imem, dmem), opFilePath(ioDir + "\\StateResult_SS.txt") {
			// Initialize PC to 0
			state.IF.PC = bitset<32>(0);
			state.IF.nop = false;
			nextState = state;
		}

		void step() {
			/* Your implementation*/
			if (halted) {
				return;
			}

			// Initialize nextState
			nextState = state;

			// Fetch instruction
			bitset<32> instruction = ext_imem.readInstr(state.IF.PC);

			// Check for HALT instruction (we'll define 0xFFFFFFFF as HALT)
			if (instruction.to_ulong() == 0xFFFFFFFF) {
				halted = true;
				myRF.outputRF(cycle);
				printState(state, cycle);
				cycle++;
			}

			instruction_count++;

			// Decode instruction
			bitset<7> opcode = bitset<7>(instruction.to_ulong() & 0x7F);
			unsigned long opcode_ulong = opcode.to_ulong();

			// Handle different instruction types
			if (!halted){

				if (opcode_ulong == 0x33) {
					executeRType(instruction);
				}
				else if (opcode_ulong == 0x13) {
					executeIType(instruction);
				}
				else if (opcode_ulong == 0x03) {
					executeLoad(instruction);
				}
				else if (opcode_ulong == 0x23) {
					executeStore(instruction);
				}
				else if (opcode_ulong == 0x63) {
					executeBranch(instruction);
				}
				else if (opcode_ulong == 0x6F) {
					executeJAL(instruction);
				}
				else {
					cout << "Unsupported instruction at PC: " << state.IF.PC.to_ulong() << endl;
					halted = true;
				}
			}

			// Output register file and state
			myRF.outputRF(cycle);
			printState(nextState, cycle);

			// Update state and cycle count
			state = nextState;
			cycle++;

		}

		void executeRType(bitset<32> instruction) {
			// Extract fields
			bitset<3> funct3 = bitset<3>((instruction.to_ulong() >> 12) & 0x7);
			bitset<7> funct7 = bitset<7>((instruction.to_ulong() >> 25) & 0x7F);
			bitset<5> rs1 = bitset<5>((instruction.to_ulong() >> 15) & 0x1F);
			bitset<5> rs2 = bitset<5>((instruction.to_ulong() >> 20) & 0x1F);
			bitset<5> rd = bitset<5>((instruction.to_ulong() >> 7) & 0x1F);

			// Read operands
			bitset<32> operand1 = myRF.readRF(rs1);
			bitset<32> operand2 = myRF.readRF(rs2);

			bitset<32> result;

			// Execute operation based on funct3 and funct7
			if (funct3.to_ulong() == 0x0) {
				if (funct7.to_ulong() == 0x00) { // ADD
					result = bitset<32>(operand1.to_ulong() + operand2.to_ulong());
				}
				else if (funct7.to_ulong() == 0x20) { // SUB
					result = bitset<32>(operand1.to_ulong() - operand2.to_ulong());
				}
				else {
					cout << "Unsupported funct7 in R-type at PC: " << state.IF.PC.to_ulong() << endl;
					halted = true;
					return;
				}
			}
			else if (funct3.to_ulong() == 0x4) { // XOR
				result = operand1 ^ operand2;
			}
			else if (funct3.to_ulong() == 0x6) { // OR
				result = operand1 | operand2;
			}
			else if (funct3.to_ulong() == 0x7) { // AND
				result = operand1 & operand2;
			}
			else {
				cout << "Unsupported funct3 in R-type at PC: " << state.IF.PC.to_ulong() << endl;
				halted = true;
				return;
			}

			// Write result
			myRF.writeRF(rd, result);

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
		}

		void executeIType(bitset<32> instruction) {
			// Extract fields
			bitset<3> funct3 = bitset<3>((instruction.to_ulong() >> 12) & 0x7);
			bitset<5> rs1 = bitset<5>((instruction.to_ulong() >> 15) & 0x1F);
			bitset<5> rd = bitset<5>((instruction.to_ulong() >> 7) & 0x1F);

			// Extract immediate and sign-extend
			int32_t imm = (instruction.to_ulong() >> 20) & 0xFFF;
			if (imm & 0x800) {
				imm |= 0xFFFFF000;
			}

			// Read operand
			bitset<32> operand1 = myRF.readRF(rs1);

			bitset<32> result;

			// Execute operation based on funct3
			if (funct3.to_ulong() == 0x0) { // ADDI
				result = bitset<32>(operand1.to_ulong() + imm);
			}
			else if (funct3.to_ulong() == 0x4) { // XORI
				result = bitset<32>(operand1.to_ulong() ^ imm);
			}
			else if (funct3.to_ulong() == 0x6) { // ORI
				result = bitset<32>(operand1.to_ulong() | imm);
			}
			else if (funct3.to_ulong() == 0x7) { // ANDI
				result = bitset<32>(operand1.to_ulong() & imm);
			}
			else {
				cout << "Unsupported funct3 in I-type at PC: " << state.IF.PC.to_ulong() << endl;
				halted = true;
				return;
			}

			// Write result
			myRF.writeRF(rd, result);

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
		}

		void executeLoad(bitset<32> instruction) {
			// Extract fields
			// Not extracting funct3 because we are only using LW, it has a constant funct3

			bitset<5> rs1 = bitset<5>((instruction.to_ulong() >> 15) & 0x1F);
			bitset<5> rd = bitset<5>((instruction.to_ulong() >> 7) & 0x1F);

			// Extract immediate and sign-extend
			int32_t imm = (instruction.to_ulong() >> 20) & 0xFFF;
			if (imm & 0x800) {
				imm |= 0xFFFFF000;
			}

			// Read base address
			bitset<32> base_addr = myRF.readRF(rs1);

			// Compute effective address
			bitset<32> mem_addr = bitset<32>(base_addr.to_ulong() + imm);

			// Read from data memory
			bitset<32> load_data = ext_dmem.readDataMem(mem_addr);

			// Write result
			myRF.writeRF(rd, load_data);

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
		}

		void executeStore(bitset<32> instruction) {
			// Extract fields
			// Not extracting funct3 because we are only using SW, it has a constant funct3

			bitset<5> rs1 = bitset<5>((instruction.to_ulong() >> 15) & 0x1F);
			bitset<5> rs2 = bitset<5>((instruction.to_ulong() >> 20) & 0x1F);

			// Extract immediate and sign-extend
			int32_t imm = (((instruction.to_ulong() >> 25) & 0x7F) << 5) |
				((instruction.to_ulong() >> 7) & 0x1F);
			if (imm & 0x800) {
				imm |= 0xFFFFF000;
			}

			// Read base address and data to store
			bitset<32> base_addr = myRF.readRF(rs1);
			bitset<32> store_data = myRF.readRF(rs2);

			// Compute effective address
			bitset<32> mem_addr = bitset<32>(base_addr.to_ulong() + imm);

			// Write to data memory
			ext_dmem.writeDataMem(mem_addr, store_data);

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
		}

		void executeBranch(bitset<32> instruction) {
			// Extract fields
			bitset<3> funct3 = bitset<3>((instruction.to_ulong() >> 12) & 0x7);
			bitset<5> rs1 = bitset<5>((instruction.to_ulong() >> 15) & 0x1F);
			bitset<5> rs2 = bitset<5>((instruction.to_ulong() >> 20) & 0x1F);

			// Extract immediate and sign-extend
			int32_t imm = (((instruction.to_ulong() >> 31) & 0x1) << 12) |
				(((instruction.to_ulong() >> 7) & 0x1) << 11) |
				(((instruction.to_ulong() >> 25) & 0x3F) << 5) |
				(((instruction.to_ulong() >> 8) & 0xF) << 1);
			if (imm & 0x1000) {
				imm |= 0xFFFFE000;
			}
			//imm <<= 1; // Branch addresses are word-aligned

			// Read operands
			bitset<32> operand1 = myRF.readRF(rs1);
			bitset<32> operand2 = myRF.readRF(rs2);

			bool take_branch = false;

			if (funct3.to_ulong() == 0x0) { // BEQ
				take_branch = (operand1 == operand2);
			}
			else if (funct3.to_ulong() == 0x1) { // BNE
				take_branch = (operand1 != operand2);
			}
			else {
				cout << "Unsupported funct3 in Branch at PC: " << state.IF.PC.to_ulong() << endl;
				halted = true;
				return;
			}

			// Update PC
			if (take_branch) {
				nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + imm);
			}
			else {
				nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
			}
		}

		void executeJAL(bitset<32> instruction) {
			// Extract rd
			bitset<5> rd = bitset<5>((instruction.to_ulong() >> 7) & 0x1F);

			// Extract immediate and sign-extend
			int32_t imm = (((instruction.to_ulong() >> 31) & 0x1) << 20) |
				(((instruction.to_ulong() >> 12) & 0xFF) << 12) |
				(((instruction.to_ulong() >> 20) & 0x1) << 11) |
				(((instruction.to_ulong() >> 21) & 0x3FF) << 1);
			if (imm & 0x100000) {
				imm |= 0xFFF00000;
			}
			//imm <<= 1; // Jump addresses are word-aligned

			// Compute return address
			bitset<32> return_addr = bitset<32>(state.IF.PC.to_ulong() + 4);

			// Write return address to rd
			myRF.writeRF(rd, return_addr);

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + imm);
		}


		void printState(stateStruct state, int cycle) {
    		ofstream printstate;
			if (cycle == 0)
				printstate.open(opFilePath, std::ios_base::trunc);
			else
    			printstate.open(opFilePath, std::ios_base::app);
    		if (printstate.is_open()) {
				printstate << "----------------------------------------------------------------------" << endl;
    		    printstate<<"State after executing cycle: " << cycle << endl;

    		    printstate << "IF.PC: " << state.IF.PC.to_ulong() <<endl;
    		    printstate << "IF.nop: " << (halted ? "True" : "False") << endl;
    		}
    		else cout<<"Unable to open SS StateResult output file." << endl;
    		printstate.close();
		}
	private:
		string opFilePath;
};


int main(int argc, char* argv[]) {
	
	string ioDir = "";
    if (argc == 1) {
        cout << "Enter path containing the memory files: ";
        cin >> ioDir;
    }
    else if (argc > 2) {
        cout << "Invalid number of arguments. Machine stopped." << endl;
        return -1;
    }
    else {
        ioDir = argv[1];
        cout << "IO Directory: " << ioDir << endl;
    }

    InsMem imem = InsMem("Imem", ioDir);
    DataMem dmem_ss = DataMem("SS", ioDir);

	SingleStageCore SSCore(ioDir, imem, dmem_ss);

	while (!SSCore.halted) {
		SSCore.step();
	}

	// Dump data memory
	dmem_ss.outputDataMem();

	// Compute performance metrics
    uint32_t total_cycles = SSCore.cycle;
    uint32_t total_instructions = SSCore.instruction_count;
    double average_CPI = static_cast<double>(total_cycles) / total_instructions;
    double IPC = static_cast<double>(total_instructions) / total_cycles;

    // Write performance metrics
    ofstream perf_metrics_file;
    perf_metrics_file.open(ioDir + "\\PerformanceMetrics_Result.txt", std::ios_base::trunc);
    if (perf_metrics_file.is_open()) {
		perf_metrics_file << "-----------------------------Single Stage Core Performance Metrics-----------------------------" << endl;
        perf_metrics_file << "Number of cycles taken: " << total_cycles << endl;
        perf_metrics_file << "Total Number of Instructions: " << total_instructions << endl;
        perf_metrics_file << "Cycles per instruction: " << average_CPI << endl;
        perf_metrics_file << "Instructions per cycle: " << IPC << endl;
    }
    else {
        cout << "Unable to open PerformanceMetrics_Result.txt for writing." << endl;
    }
    perf_metrics_file.close();

	return 0;
}
