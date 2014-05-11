typedef struct s_trace_buffer
{
	int x, y;
	int iblk;
	int data_pin;
	int usage;
	int capacity;
	int *sink_inodes;
} t_trace_buffer;

boolean inc_trace(int *trace_nets,
		int num_trace_nets,
		int new_num_nets,
		t_trace_buffer *tb,
		int num_tb,
		struct s_router_opts *router_opts, 
		struct s_det_routing_arch det_routing_arch,
		t_timing_inf timing_inf,
		struct s_file_name_opts *FileNameOpts,
		t_ivec **clb_opins_used_locally,
		float **net_slack,
		float T_crit);

int inc_parse_trace_string(char *trace_string, int **trace_nets, int *new_num_nets);

void inc_connect_net(int inet, int iblk, int ptc);

int inc_reclaim_tbs(t_trace_buffer **tb, const t_arch *arch);

void inc_assign_trace_nets(int *trace_nets,
			int num_trace_nets,
			t_trace_buffer *tb,
			int num_tb,
			float *criticality,
			int **trace_iblk,
			int **trace_ptc);
