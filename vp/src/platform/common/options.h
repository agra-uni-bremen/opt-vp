#ifndef RISCV_VP_OPTIONS_H
#define RISCV_VP_OPTIONS_H

#include "trace/config.h"
#include "util/options.h"

#include <boost/program_options.hpp>
#include <iostream>

class Options : public boost::program_options::options_description {
public:
	Options(void);
	virtual ~Options();
	virtual void parse(int argc, char **argv);

	std::string input_program;
	std::string output_file;

	// tracing. Read these through trace_config() rather than one by one: every platform
	// applies the whole set with core.tracer.configure(opt.trace_config()).
	std::string coverage_csv_file;
	unsigned int top_n = 10;
	float similarity_threshold = 0.2f;
	//effective instruction tree depth, 0 = use the depth the VP was compiled with
	unsigned int instruction_tree_depth = 0;
	//dot only: omit branches below this share of their tree's weight, 0 draws every branch
	float graph_branch_threshold = 0.05f;
	std::string scoring_library = "./vp/build/lib/libfunctions.so";
	bool output_as_dot = false;
	bool output_as_json = false;
	bool output_as_csv = false;
	bool output_full_export = false;
	bool interactive_mode = false;

	bool suppress_prompts = false;

	bool intercept_syscalls = false;
	bool error_on_zero_traphandler = false;
	bool use_debug_runner = false;
	unsigned int debug_port = 5005;
	bool trace_mode = false;
	unsigned int tlm_global_quantum = 10;
	bool use_instr_dmi = false;
	bool use_data_dmi = false;

	/**
	 * The tracing settings as the trace library wants them.
	 *
	 * `hart_id` is zero unless the platform has more than one core, in which case pass each
	 * core's id so their exported files do not collide.
	 */
	TraceConfig trace_config(unsigned int hart_id = 0) const;

	virtual void printValues(std::ostream& os = std::cout) const;

protected:
	void add_memory_options(unsigned int &mem_start_addr, unsigned int &mem_size);
	void add_quiet_option(bool &quiet);
	void add_use_e_base_isa_option(bool &use_E_base_isa);
	void add_entry_point_option(OptionValue<unsigned long> &entry_point);

private:

	boost::program_options::positional_options_description pos;
	boost::program_options::variables_map vm;
};


#endif
