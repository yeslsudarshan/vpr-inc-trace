#include <stdio.h>
#include "vpr_types.h"
#include "globals.h"
#include "route_common.h"
#include "check_netlist.h"
#include "check_route.h"
#include "cluster_legality.h"
#include "inc_trace.h"
#include "inc_route.h"
#include "inc_route_bfs.h"
#include "inc_route_dir.h"
#include "inc_misc.h"
#include <assert.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <float.h>

/**
 * Finds the index of the "memory" type (returned value)
 * and populates the data_pin and data_width params
 */
static int inc_find_mem(int *data_pin, int *data_width)
{
	int i;

	/* Go through all the types looking for the name "memory" */
	for (i = 0; i < num_types; i++) 
	{
		if (strcmp(type_descriptors[i].name, "memory") == 0) 
		{
			int j, type, pin;
			type = i;
			pin = 0;
			/* Go through each port looking for the name "data" */
			for (j = 0; j < type_descriptors[i].pb_type->num_ports; j++)
			{
				if (strcmp(type_descriptors[i].pb_type->ports[j].name, "data") == 0)
				{
					*data_pin = pin;
					*data_width = type_descriptors[i].pb_type->ports[j].num_pins;
				}
				else
					pin += type_descriptors[i].pb_type->ports[j].num_pins;
			}
			return type;
		}
	}
	printf(ERRTAG "\"memory\" type not found!");
	exit(1);
}

/**
 * Wrapper function for alloc_and_load_rr_graph_for_pb_garph_node()
 * which clobbers the rr_node global
 */
static void inc_alloc_and_load_rr_graph_for_pb_graph_node(	INP t_pb_graph_node *pb_graph_node, 
		INP const t_arch* arch, 
		int mode, 
		t_rr_node *local_rr_node) 
{
	t_rr_node *global_rr_node;

	global_rr_node = rr_node;
	rr_node = local_rr_node;
	alloc_and_load_rr_graph_for_pb_graph_node(pb_graph_node, arch, mode);
	rr_node = global_rr_node;
}


/**
 * Place a new block into the circuit, returning its index
 */
static int inc_place_block(short x, short y, int type, const t_arch *arch)
{
	int iblk;
	int i;
	iblk = num_blocks;
	num_blocks++;
	block = (struct s_block*)realloc(block, num_blocks*sizeof(struct s_block));
	block[iblk].name = NULL;
	block[iblk].x = x;
	block[iblk].y = y;
	block[iblk].z = 0;
	block[iblk].type = &type_descriptors[type];
	block[iblk].pb = calloc(1, sizeof(t_pb));
	block[iblk].pb->pb_graph_node = block[iblk].type->pb_graph_head;
	block[iblk].pb->rr_graph = calloc(block[iblk].type->pb_graph_head->total_pb_pins, sizeof(t_rr_node));
	inc_alloc_and_load_rr_graph_for_pb_graph_node(block[iblk].pb->pb_graph_node, arch, 0, block[iblk].pb->rr_graph);
	block[iblk].nets = malloc(block[iblk].type->num_pins*sizeof(int));
	for (i = 0; i < block[iblk].type->num_pins; i++)
	{
		block[iblk].nets[i] = OPEN;
	}

	assert(grid[block[iblk].x][block[iblk].y].usage < grid[block[iblk].x][block[iblk].y].type->capacity);
	grid[block[iblk].x][block[iblk].y].blocks[grid[block[iblk].x][block[iblk].y].usage] = iblk;
	grid[block[iblk].x][block[iblk].y].usage++;

	return iblk;
}

/**
 * Reclaims all spare memory blocks as trace-buffers
 * Populate the tb parameter with their data
 */
int inc_reclaim_tbs(t_trace_buffer **tb, const t_arch *arch)
{
	int mem_type, data_pin, data_width;
	int i, num_tb;

	mem_type = inc_find_mem(&data_pin, &data_width);
	num_tb = 0;

	/* Search the FPGA grid for unused memory blocks,
	 * and place a trace-buffer into those */
	for (i = 0; i <= nx+1; i++) 
	{
		int j;
		for (j = 0; j <= ny+1; j++)
		{
			if (grid[i][j].offset == 0 && grid[i][j].type->index == mem_type)
			{
				if (grid[i][j].usage == 0)
				{
					*tb = realloc(*tb, sizeof(**tb) * (num_tb+1));
					(*tb)[num_tb].x = i; 
					(*tb)[num_tb].y = j; 
					(*tb)[num_tb].iblk = num_blocks;
					(*tb)[num_tb].data_pin = data_pin;
					(*tb)[num_tb].usage = 0;
					(*tb)[num_tb].capacity = data_width;
					(*tb)[num_tb].sink_inodes = NULL;
					inc_place_block(i, j, mem_type, arch);
					num_tb++;
				}
			}
		}
	}

	if (num_tb == 0)
	{
		printf(ERRTAG "No free memory blocks to reclaim as trace-buffers!");
		exit(1);
	}
	return num_tb;
}

/**
 * Mark all trace-buffer "data" nodes as targets
 * (NB: I'm using target_flag as an indicator for which
 * trace-buffer it belongs to)
 * Adapted from mark_ends() 
 * */
static void inc_mark_targets(int num_tb, t_trace_buffer *tb)
{
	int inode;
	int i, j;
	for (i = 0; i < num_tb; i++) 
	{
		for (j = 0; j < tb[i].capacity; j++) 
		{
			inode = get_rr_node_index(tb[i].x, tb[i].y, SINK, 
					j+tb[i].data_pin, rr_node_indices);
			assert(rr_node_route_inf[inode].target_flag == 0);
			rr_node_route_inf[inode].target_flag = tb[i].iblk;
		}
	}
}

/**
 * Find the block corresponding to the x/y coordinates given
 */
static int inc_find_block(short x, short y)
{
	int i = 0;
	for (i = 0; i < num_blocks; i++)
	{
		if (block[i].x == x && block[i].y == y)
		{
			return i;
		}
	}
	return -1;
}

/**
 * Connect the specified net to the block/pin specified 
 */
void inc_connect_net(int inet, int iblk, int ptc)
{
	int isink;
	int node1, node2;

	assert(iblk >= 0);

	clb_net[inet].num_sinks++;
	isink = clb_net[inet].num_sinks;
	if (clb_net[inet].node_block[isink] != OPEN)
		assert(clb_net[inet].node_block[isink] == iblk);
	else
		clb_net[inet].node_block[isink] = iblk;
	assert(clb_net[inet].node_block_port == NULL);
	clb_net[inet].node_block_pin = realloc(clb_net[inet].node_block_pin, (isink+1)*sizeof(int));
	clb_net[inet].node_block_pin[isink] = ptc;

	assert(block[iblk].nets[ptc] == OPEN);
	block[iblk].nets[ptc] = inet;

	assert(block[iblk].pb->rr_graph[ptc].net_num == OPEN);
	block[iblk].pb->rr_graph[ptc].net_num = clb_to_vpack_net_mapping[inet];

	/* Use mode zero (which in sample_arch.xml is the widest) */
	/* node1 -- interconnect */
	node1 = block[iblk].pb->rr_graph[ptc].edges[0];
	assert(node1 == block[iblk].pb->rr_graph[ptc].pb_graph_pin->output_edges[0]->output_pins[0]->pin_count_in_cluster);
	assert(block[iblk].pb->rr_graph[node1].prev_node == OPEN);
	block[iblk].pb->rr_graph[node1].prev_node = ptc;
	assert(block[iblk].pb->rr_graph[node1].prev_edge == OPEN);
	block[iblk].pb->rr_graph[node1].prev_edge = 0;
	assert(block[iblk].pb->rr_graph[node1].net_num == OPEN);
	block[iblk].pb->rr_graph[node1].net_num = clb_to_vpack_net_mapping[inet];
	assert(block[iblk].pb->rr_graph[node1].num_edges == 1);

	/* node2 -- sink */
	node2 = block[iblk].pb->rr_graph[node1].edges[0];
	assert(block[iblk].pb->rr_graph[node2].type == SINK);
	assert(block[iblk].pb->rr_graph[node2].prev_node == OPEN);
	block[iblk].pb->rr_graph[node2].prev_node = node1;
	assert(block[iblk].pb->rr_graph[node2].prev_edge == OPEN);
	block[iblk].pb->rr_graph[node2].prev_edge = 0;
	assert(block[iblk].pb->rr_graph[node2].net_num == OPEN);
	block[iblk].pb->rr_graph[node2].net_num = clb_to_vpack_net_mapping[inet];
}

/**
 * Connect all trace nets to their trace_tail sinks 
 */
static void inc_connect_trace(int *trace_nets, int num_trace_nets, int old_num_nets, boolean inc_many_to_many)
{
	int i, j;
	int inet, sink_inode, inode;
	int iblk, ptc;
	int isink;

	for (i = 0; i  < num_trace_nets; i++) 
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		sink_inode = trace_tail[inet]->index;
		assert(rr_node[sink_inode].type == SINK);
		iblk = inc_find_block(rr_node[sink_inode].xlow, rr_node[sink_inode].ylow);
		ptc = rr_node[sink_inode].ptc_num;
		inc_connect_net(inet, iblk, ptc);

		isink = clb_net[inet].num_sinks;

		/* Update net_rr_terminals if its empty, or if many-to-many 
		 * flexibility is enabled */
		if (net_rr_terminals[inet][isink] == OPEN || inc_many_to_many)
			net_rr_terminals[inet][isink] = sink_inode;
		else
			assert(net_rr_terminals[inet][isink] == sink_inode);

		/* Fix wrong class error in check_source() 
		 * for newly formed (global) inets */
		if (inet >= old_num_nets)
		{
			int x, y;
			t_type_ptr type;
			inode = trace_head[inet]->index;
			x = rr_node[inode].xlow;
			y = rr_node[inode].ylow;
			type = grid[x][y].type;
			for (j = 0; j < type->num_pins; j++)
			{
				if (type->pin_class[j] == rr_node[inode].ptc_num)
				{
					clb_net[inet].node_block_pin[0] = j;
					break;
				}
			}
			assert(j < type->num_pins);
		}
	}
}

/**
 * Check that the OPIN of inet is legitimate
 */
static boolean inc_feasible_outpin(	int inet,
									int old_num_nets)
{
	int iblk, ipin, vnet, ptc;
	int ible, nbles;
	int inode, source_inode;
	int ninpins, noutpins;
	iblk = clb_net[inet].node_block[0];
	ipin = clb_net[inet].node_block_pin[0];

	vnet = block[iblk].pb->rr_graph[ipin].net_num;
	assert(vpack_to_clb_net_mapping[vnet] == inet);

	assert(block[iblk].pb->pb_graph_node->pb_type->num_pb == 1);
	assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);
	assert(block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children == 1);
	nbles = block[iblk].pb->pb_graph_node->pb_type->modes[0].pb_type_children[0].num_pb;
	assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_ports == 1);
	ninpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_pins[0];
	assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_ports == 1);
	noutpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_pins[0];

	assert(inet >= old_num_nets);
	source_inode = trace_head[inet]->index;

	for (ible = 0; ible < nbles; ible++)
	{
		int used_inputs;
		boolean fractured;

		assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_ports == 1);
		assert(ninpins == block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_pins[0]);

		/* Count how many used inputs there are in its BLE */
		used_inputs = 0;
		for (ipin = 0; ipin < ninpins; ipin++)
		{
			ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->input_pins[0][ipin]
				.pin_count_in_cluster;
			vnet = block[iblk].pb->rr_graph[ptc].net_num;
			if (vnet != OPEN)
			{
				used_inputs++;
			}
		}
		/* If we are not using all the input pins, then this net can 
		 * be considered fractured ... because in the k6_N10_memDepth16384...
		 * arch file both fractured 5LUTs use the same inputs */
		/* FIXME: Make this more generic */
		fractured = used_inputs < ninpins;

		/* Find the CLB OPIN that this inet originates from */
		inode = OPEN;
		for (ipin = 0; ipin < noutpins; ipin++)
		{
			/* BLE OPIN ptc */
			ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
				.pin_count_in_cluster;
			vnet = block[iblk].pb->rr_graph[ptc].net_num;
			/* If this net is a local net (has no global net, or has a global net which we created for tracing) */
			if (vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets) 
			{
				assert(block[iblk].pb->pb_graph_node->num_output_ports == 1);
				/* CLB OPIN */
				ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*noutpins+ipin].pin_count_in_cluster;

				/* Get the rr_node index of CLB_OPIN SOURCE */
				inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
						SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

				if (source_inode == inode)
				{
					break;
				}
			}
		}

		/* If this is the BLE */
		if (source_inode == inode)
		{
			int ineligible;
			ineligible = 0;
			/* Make sure there's at most one ineligible output pin for
			 * fractured, but none for unfractured */
			for (ipin = 0; ipin < noutpins; ipin++)
			{
				ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
					.pin_count_in_cluster;
				vnet = block[iblk].pb->rr_graph[ptc].net_num;

				if (!(vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets))
				{
					ineligible++;
				}
			}

			if ((fractured && ineligible > 1) || (!fractured && ineligible > 0))
			{
				return FALSE;
			}
			return TRUE;
		}
	}
	assert(ible < nbles);
	return TRUE;
}


/**
 * Check if just the incremental routing is feasible 
 * Also, track how many nets and nodes were overused
 * Adapted from feasible_routing()
 */
static boolean inc_feasible_routing(int num_trace_nets, 
		int *trace_nets, 
		struct s_trace **old_trace_tail, 
		int *nets_overuse, 
		int *nodes_overuse,
		int old_num_nets)
{
	int i, inet, inode;
	struct s_trace *tptr;
	boolean net_overuse;
	*nets_overuse = 0;
	*nodes_overuse = 0;

	/* Go through each trace net in turn */
	for (i = 0; i < num_trace_nets; i++)
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		/* Find when the incremental trace starts */
		if (old_trace_tail[inet])
			tptr = old_trace_tail[inet]->next->next;
		else
			tptr = trace_head[inet];

		/* Check that each incremental node does
		 * not exceed occupancy */
		net_overuse = FALSE;
		while (tptr)
		{
			inode = tptr->index;
			if (rr_node[inode].occ > rr_node[inode].capacity)
			{
				(*nodes_overuse)++;
				net_overuse = TRUE;
			}
			tptr = tptr->next;
		}

		/* If this is a new global net, then
		 * check that its OPIN is feasible too */
		if (inet >= old_num_nets)
		{
			if (!inc_feasible_outpin(inet, old_num_nets))
			{
				(*nodes_overuse)++;
				net_overuse = TRUE;
			}
		}

		if (net_overuse)
			(*nets_overuse)++;
	}

	/* To be extra safe, double-check using the vanilla 
	 * feasible_routing() */
	assert(feasible_routing() == (*nodes_overuse == 0));

	return (*nodes_overuse == 0);
}

/**
 * inc_update_cost() penalizes incremental nets
 * that have infeasible OPINs 
 */
static void inc_update_cost(int num_trace_nets, 
		int *trace_nets, 
		int old_num_nets,
		float acc_fac)
{
	int i, inet, inode;
	for (i = 0; i < num_trace_nets; i++)
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		/* If this is a new global net */
		if (inet >= old_num_nets)
		{
			inode = trace_head[inet]->index;
			if (!inc_feasible_outpin(inet, old_num_nets))
			{
				/* Penalize as if it was over capacity by one */
				rr_node_route_inf[inode].acc_cost += acc_fac;
			}
		}
	}
}

/**
 * Undoes what inc_add_clb_net() does 
 */
static void inc_remove_clb_net(int inet)
{
	int iblk, ptc;

	/* Restore all data structures */
	assert(trace_head[inet] == NULL);
	assert(trace_tail[inet] == NULL);
	assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[inet]] == inet);
	vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[inet]] = OPEN;
	iblk = clb_net[inet].node_block[0];
	ptc = clb_net[inet].node_block_pin[0];
	assert(block[iblk].pb->rr_graph[ptc].net_num = clb_to_vpack_net_mapping[inet]);
	block[iblk].pb->rr_graph[ptc].net_num = OPEN;
	assert(block[iblk].pb->rr_graph[ptc].prev_node != OPEN);
	block[iblk].pb->rr_graph[ptc].prev_node = OPEN;
	assert(block[iblk].pb->rr_graph[ptc].prev_edge != OPEN);
	block[iblk].pb->rr_graph[ptc].prev_edge = OPEN;
}

/**
 * Resolve congestion by iteratively (according to inet) 
 * discarding each net if it has any over-used resources,
 * until a valid solution exists
 */
static void inc_resolve_congestion(int num_trace_nets, 
		int *trace_nets, 
		struct s_trace **old_trace_tail,
		int old_num_nets,
		int *num_failed_nets,
		float pres_fac)
{
	int i, inet, inode;
	struct s_trace *tptr;
	boolean global, net_overuse;

	/* Go through each trace net */
	for (i = 0; i < num_trace_nets; i++)
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		global = inet < old_num_nets;
		if (global)
			tptr = old_trace_tail[inet]->next->next;
		else
			tptr = trace_head[inet];

		/* Find out if net is overused */
		net_overuse = FALSE;
		while (tptr)
		{
			inode = tptr->index;
			if (rr_node[inode].occ > rr_node[inode].capacity)
			{
				net_overuse = TRUE;
			}
			tptr = tptr->next;
		}

		/* If net is overused, or if it is a new global net with
		 * an illegal OPIN */
		if (net_overuse || (!global && !inc_feasible_outpin(inet, old_num_nets)))
		{
			fprintf(stdout, "Abandoning trace_nets[%d]: vnet=%d inet=%d due to congestion!\n", i, clb_to_vpack_net_mapping[inet], inet);
			if (global)
			{
				/* Only remove the incremental (and not the user) trace */
				assert(old_trace_tail[inet]->next);
				/*if (old_trace_tail[inet]->next)*/
				{
					inc_update_one_cost(old_trace_tail[inet]->next->next, -1);
					pathfinder_update_one_cost(old_trace_tail[inet]->next->next, -1, pres_fac);
				}
				inc_free_traceback(inet, old_trace_tail);
				assert(trace_tail[inet] == old_trace_tail[inet]);
				assert(trace_tail[inet]->next == NULL);
			}
			else
			{
				inc_update_one_cost(trace_head[inet], -1);
				pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
				free_traceback(inet);

				inc_remove_clb_net(inet);
			}		
			trace_nets[i] = OPEN;
			(*num_failed_nets)++;
		}
	}

	/* Double check that everything is feasible now */
	assert(feasible_routing());
}

/**
 * Convert a vpack (local) net into a new clb (global) net
 */
static int inc_add_clb_net(int vnet, int num_nets)
{
	int iblk, ptc, inode, i, clb_inet;
	/* Create a new clb_net element */
	clb_net = realloc(clb_net, (num_nets+1)*sizeof(struct s_net));
	clb_net[num_nets].name = (char*)strdup(vpack_net[vnet].name);
	clb_net[num_nets].num_sinks = 0;
	clb_net[num_nets].node_block = malloc(sizeof(int));
	clb_net[num_nets].node_block[0] = iblk = vpack_net[vnet].node_block[0];
	assert(iblk != OPEN);
	clb_net[num_nets].node_block_port = NULL;
	clb_net[num_nets].node_block_pin = malloc(sizeof(int));
	clb_net[num_nets].node_block_pin[0] = ptc = vpack_net[vnet].node_block_pin[0];
	assert(ptc != OPEN);
	clb_net[num_nets].is_global = FALSE;
	clb_net[num_nets].is_const_gen = FALSE;
	/* Create a new clb_to_vpack_mapping element */
	clb_to_vpack_net_mapping = realloc(clb_to_vpack_net_mapping, (num_nets+1)*sizeof(int));
	clb_to_vpack_net_mapping[num_nets] = vnet;

	/* CLB_OPIN ptc */
	assert(block[iblk].pb->rr_graph[ptc].net_num == OPEN);
	block[iblk].pb->rr_graph[ptc].net_num = vnet;
	assert(block[iblk].pb->rr_graph[ptc].pb_graph_pin->num_input_edges == 1);
	assert(block[iblk].pb->rr_graph[ptc].pb_graph_pin->input_edges[0]->num_input_pins == 1);
	inode = block[iblk].pb->rr_graph[ptc].pb_graph_pin->input_edges[0]
		->input_pins[0]->pin_count_in_cluster;
	assert(block[iblk].pb->rr_graph[ptc].prev_node == OPEN);
	block[iblk].pb->rr_graph[ptc].prev_node = inode;
	assert(block[iblk].pb->rr_graph[inode].net_num == vnet);

	/* Enable the edge that connects the previous inode (BLE_OPIN) to this CLB_OPIN */
	for (i = 0; i < block[iblk].pb->rr_graph[inode].num_edges; i++)
	{
		if (block[iblk].pb->rr_graph[inode].edges[i] == ptc)
			break;
	}
	assert(i < block[iblk].pb->rr_graph[inode].num_edges);
	assert(block[iblk].pb->rr_graph[ptc].prev_edge == OPEN);
	block[iblk].pb->rr_graph[ptc].prev_edge = i;

	clb_inet = num_nets;

	return clb_inet;
}

/**
 * Parse the trace_string into the trace_nets[] array,
 * transforming local nets into new global ones if necessary
 */
int inc_parse_trace_string(char *trace_string, int **trace_nets, int *new_num_nets)
{
	char *pch;
	int inet, vnet, num_trace_nets;

	*trace_nets = malloc(sizeof(int));
	*new_num_nets = num_nets;
	num_trace_nets = 0;

	pch = (char*)strtok(trace_string, ",");
	while (pch != NULL)
	{
		vnet = atoi(pch);

		/* If net is local, make it global */
		if (vpack_to_clb_net_mapping[vnet] == OPEN)
		{
			inet = inc_add_clb_net(vnet, *new_num_nets);
			(*new_num_nets)++;
			vpack_to_clb_net_mapping[vnet] = inet;

			assert(clb_net[inet].num_sinks == 0);
			clb_net[inet].node_block = realloc(clb_net[inet].node_block, 2*sizeof(int));
			clb_net[inet].node_block[1] = OPEN;

			/* Create a new net_rr_terminals if appropriate 
			 * (during post-map) */
			if (net_rr_terminals)
			{
				int iblk, ptc, inode;
				iblk = clb_net[inet].node_block[0];
				ptc = clb_net[inet].node_block_pin[0];
				inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
						SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

				net_rr_terminals = realloc(net_rr_terminals, (*new_num_nets)*sizeof(int *));
				net_rr_terminals[inet] = malloc(2*sizeof(int));
				net_rr_terminals[inet][0] = inode;
				net_rr_terminals[inet][1] = OPEN;
			}
		}
		else
		{
			int num_sinks;

			inet = vpack_to_clb_net_mapping[vnet];
			num_sinks = clb_net[inet].num_sinks;

			clb_net[inet].node_block = realloc(clb_net[inet].node_block, (num_sinks+2)*sizeof(int));
			clb_net[inet].node_block[num_sinks+1] = OPEN;

			if (net_rr_terminals)
			{
				int *tmp;

				/* Code below doesn't work because net_rr_terminals[inet] was 
				 * allocated using my_chunk_alloc(); instead copy the entire array */
				/* net_rr_terminals[inet] = realloc(net_rr_terminals[inet], (clb_net[inet].num_sinks+1)*sizeof(int)); */
				tmp = (int*)malloc((num_sinks+2)*sizeof(int));
				memcpy(tmp, net_rr_terminals[inet], (num_sinks+1)*sizeof(int));
				net_rr_terminals[inet] = tmp;
				net_rr_terminals[inet][num_sinks+1] = OPEN;
			}
		}

		/* Add to trace_nets[] */
		*trace_nets = realloc(*trace_nets, (num_trace_nets+1) * sizeof(int));
		(*trace_nets)[num_trace_nets] = inet;
		num_trace_nets++;

		pch = (char*)strtok(NULL, ",");
	}
	return num_trace_nets;
}

/**
 * Try and iteratively route the trace nets
 * Adapted from try_<algo>_route()
 */
static boolean inc_try_trace(
		int *trace_nets,
		int num_trace_nets, 
		int old_num_nets,
		struct s_router_opts *router_opts, 
		struct s_file_name_opts *FileNameOpts,
		struct s_det_routing_arch det_routing_arch,
		t_timing_inf timing_inf,
		t_trace_buffer *tb,
		int num_tb,
		t_ivec **clb_opins_used_locally,
		float *criticality)
{
	int inet;
	float pres_fac;
	float bend_cost;
	boolean success;
	int i, j;
	clock_t begin, end;
	int nets_overuse, nodes_overuse;
	int num_failed_nets;
	struct s_trace **old_trace_tail;

	printf("Performing incremental-routing...\n");
	begin = clock();

	/* Reallocate trace_head and trace_tail structures */
	trace_head = realloc(trace_head, num_nets*sizeof(struct s_trace*));
	trace_tail = realloc(trace_tail, num_nets*sizeof(struct s_trace*));
	/* Copy the existing trace_tail structure  */
	old_trace_tail = (struct s_trace **)malloc(num_nets * sizeof(struct s_trace *));
	memcpy(old_trace_tail, trace_tail, num_nets * sizeof(struct s_trace *));
	/* Zero out all the new trace back structures */
	for (inet = old_num_nets; inet < num_nets; inet++)
	{
		trace_head[inet] = NULL;
		trace_tail[inet] = NULL;
		old_trace_tail[inet] = NULL;
	}
	/* Reset accumulated costs stored in rr_node_route_inf */
	inc_reset_rr_node_route_structs();

	pres_fac = router_opts->first_iter_pres_fac;
	bend_cost = router_opts->bend_cost;
	num_failed_nets = 0;

	/* For every routing iteration */
	for (i = 1; i <= router_opts->max_router_iterations; i++) 
	{
		int nsuccess = 0;
		/* For every trace net */
		for (j = 0; j < num_trace_nets; j++)
		{
			boolean global;
			inet = trace_nets[j];
			if (inet == OPEN)
				continue;
			global = inet < old_num_nets;

			/* Remove rr_node.occ */
			pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
			/* Remove rr_node.inc_occ */
			if (global)
			{
				if (old_trace_tail[inet]->next)
					inc_update_one_cost(old_trace_tail[inet]->next->next, -1);
			}
			else
			{
				inc_update_one_cost(trace_head[inet], -1);
				free_traceback(inet);
			}

			switch(router_opts->inc_router_algorithm)
			{
				case BREADTH_FIRST:
					success = inc_breadth_first_route_net(inet, bend_cost, old_trace_tail, old_num_nets, router_opts);
					break;
					/* Difference between (non) timing-driven directed search is that for
					 * non timing-driven, all net criticalities are zero */
				case DIRECTED_SEARCH:
				case TIMING_DRIVEN:
					success = inc_directed_search_route_net(inet, pres_fac, router_opts->astar_fac, bend_cost, 
							old_trace_tail, old_num_nets, router_opts, num_tb, tb, criticality[j]);
					break;
				default:
					assert(FALSE);
			}

			/* Add rr_node.occ */
			pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);
			if (success)
			{
				/* Add rr_node.inc_occ */
				nsuccess += 1;
				if (global)
					inc_update_one_cost(old_trace_tail[inet]->next->next, 1);
				else
					inc_update_one_cost(trace_head[inet], 1);
			}
			else
			{
				/* Routing this net is completely impossible: 
				 * if best effort is not enabled bail out here */
				if (!router_opts->inc_best_effort)
					goto end;

				if (global)
				{
					assert(trace_tail[inet] == old_trace_tail[inet]);
				}
				else
				{
					inc_remove_clb_net(inet);
				}

				/* Give up tracing this net permanently */
				trace_nets[j] = OPEN;
				num_failed_nets++;
			}
		}

		/* Check if incremental trace is feasible */
		success = inc_feasible_routing(num_trace_nets, trace_nets, old_trace_tail, &nets_overuse, &nodes_overuse, old_num_nets);
		if (success) goto end;

		if (i == 1)
			pres_fac = router_opts->initial_pres_fac;
		else
			pres_fac *= router_opts->pres_fac_mult;

		pres_fac = min(pres_fac, HUGE_FLOAT / 1e5);

		pathfinder_update_cost(pres_fac, router_opts->acc_fac);
		inc_update_cost(num_trace_nets, trace_nets, old_num_nets, router_opts->acc_fac);
		fprintf(stdout, "Iteration %d: %d/%d successfully traced but found %d nets over-occupying %d nodes!\n", i, nsuccess, num_trace_nets, 
				nets_overuse, nodes_overuse);
	}
	/* Remove one here because for loop would have incremented i to beyond max_router_iterations */
	i--;

	/* If congestion wasn't full resolvable after the set number of iterations,
	 * then use inc_resolve_congestion() which iteratively discards until
	 * legal solution is found */
	fprintf(stdout, "Congestion unresolved after %d iteration(s). Abandon ship...\n", i);
	inc_resolve_congestion(num_trace_nets, trace_nets, old_trace_tail, old_num_nets, &num_failed_nets, pres_fac);
	success = inc_feasible_routing(num_trace_nets, trace_nets, old_trace_tail, &nets_overuse, &nodes_overuse, old_num_nets);
	assert(success);

end:
	printf("Incremental-routing ran for %d iteration(s).\n", i);

	if (success)
	{
		/* Make sure there are no holes in clb_net;
		 * holes created by abandoning impossible nets 
		 * during the routing procedure, or during
		 * inc_resolve_congestion() */
		for (i = old_num_nets; i < num_nets; i++)
		{
			if (trace_head[i] == NULL)
			{
				assert(trace_tail[i] == NULL);
				while(trace_head[num_nets-1] == NULL)
				{
					assert(trace_tail[num_nets-1] == NULL);
					num_nets--;
				}
				if (num_nets == i)
					continue;
				assert(num_nets > i);
				assert(trace_head[num_nets-1] != NULL);
				assert(trace_tail[num_nets-1] != NULL);

				trace_head[i] = trace_head[num_nets-1];
				trace_tail[i] = trace_tail[num_nets-1];
				clb_net[i] = clb_net[num_nets-1];
				assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[num_nets-1]] == num_nets-1);
				vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[num_nets-1]] = i;
				clb_to_vpack_net_mapping[i] = clb_to_vpack_net_mapping[num_nets-1];
				assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[i]] == i);
				net_rr_terminals[i] = net_rr_terminals[num_nets-1];

				for (j = 0; j < num_trace_nets; j++)
				{
					if (trace_nets[j] == (num_nets-1))
					{
						trace_nets[j] = i;
						break;
					}
				}
				assert(j < num_trace_nets);
				num_nets--;
			}
		}

		/* Check that all nets are traced */
		for (i = 0; i < num_trace_nets; i++)
		{
			inet = trace_nets[i];
			if (inet != OPEN)
			{
				assert(inet < num_nets);
				assert(trace_head[inet] != NULL);
				assert(trace_tail[inet] != NULL);
			}
		}

		/* Connect them up */
		inc_connect_trace(trace_nets, num_trace_nets, old_num_nets, router_opts->inc_many_to_many);

		check_route(router_opts->route_type,
				det_routing_arch.num_switch,
				clb_opins_used_locally);
	}

	end = clock();
#ifdef CLOCKS_PER_SEC
	printf("Incremental-routing took %g seconds\n", (float)(end - begin) / CLOCKS_PER_SEC);
#else
	printf("Incremental-routing took %g seconds\n", (float)(end - begin) / CLK_PER_SEC);
#endif

	if (success)
		printf("Incremental-routing successful for %d nets!\n", num_trace_nets - num_failed_nets);
	else
		printf("Incremental-routing failed!\n");
	fflush(stdout);

	return success;
}

/* Assign each trace_net to a trace-buffer and pin
 * using a heuristic that gives trace_nets furthest
 * away from any trace-buffer priority on its nearest 
 * trace-buffer, after weighting by its criticality */
void inc_assign_trace_nets(int *trace_nets,
			int num_trace_nets,
			t_trace_buffer *tb,
			int num_tb,
			float *criticality,
			int **trace_iblk,
			int **trace_ptc)
{
	int inet;
	int *sort_idx;
	float *mdist, tmp_mdist;
	int *nearest_tb;
	int i, j, k, iblk, x, y, ptc, sort_i;

	sort_idx = malloc(sizeof(int)*(num_trace_nets+1));
	mdist = malloc(sizeof(float)*(num_trace_nets+1));
	nearest_tb = malloc(sizeof(int)*num_trace_nets);
	*trace_iblk = malloc(sizeof(int)*num_trace_nets);
	*trace_ptc = malloc(sizeof(int)*num_trace_nets);

	/* First sort */
	for (j = 0; j < num_trace_nets; j++)
	{
		inet = trace_nets[j];
		nearest_tb[j] = OPEN;
		(*trace_iblk)[j] = OPEN;
		(*trace_ptc)[j] = OPEN;
		if (inet != OPEN)
		{
			iblk = clb_net[inet].node_block[0];
			x = block[iblk].x;
			y = block[iblk].y;
			/* Find nearest tb with capacity */
			mdist[j+1] = FLT_MAX;
			for (k = 0; k < num_tb; k++)
			{
				if (tb[k].usage < tb[k].capacity)
				{
					tmp_mdist = abs(x-tb[k].x) + abs(y-tb[k].y);

					/* Multiply by criticality --- the higher the criticality, the higher its 
					 * effective wirelength, and the higher priority it will be given a TB */
					if (criticality)
						tmp_mdist *= max(criticality[j], 0.01);

					if (tmp_mdist < mdist[j+1])
					{
						mdist[j+1] = tmp_mdist;
						nearest_tb[j] = k;
					}
				}
			}
			assert(nearest_tb[j] >= 0);
		}
		else
		{
			mdist[j+1] = FLT_MIN;
		}
	}
	heapsort(sort_idx, mdist, num_trace_nets, 0);

	sort_i = 1;
	for (i = 0; i < num_trace_nets; i++)
	{
		j = sort_idx[sort_i]-1;
		assert(j >= 0 && j < num_trace_nets);
		inet = trace_nets[j];
		assert(inet >= 0 && inet < num_nets);
		k = nearest_tb[j];
		assert(k >= 0 && k < num_tb);
		assert(tb[k].usage < tb[k].capacity);

		iblk = tb[k].iblk;
		ptc = tb[k].data_pin + tb[k].usage;

		assert((*trace_iblk)[j] == OPEN);
		assert((*trace_ptc)[j] == OPEN);
		(*trace_iblk)[j] = iblk;
		(*trace_ptc)[j] = ptc;

		tb[k].usage++;
		sort_i++;

		/* If we've filled up an entire trace-buffer, 
		 * re-sort with just the remaining ones */
		if (tb[k].usage == tb[k].capacity)
		{
			for (j = 0; j < num_trace_nets; j++)
			{
				inet = trace_nets[j];
				nearest_tb[j] = OPEN;
				if ((*trace_iblk)[j] == OPEN)
				{
					iblk = clb_net[inet].node_block[0];
					x = block[iblk].x;
					y = block[iblk].y;
					/* Find nearest tb with capacity */
					mdist[j+1] = FLT_MAX;
					for (k = 0; k < num_tb; k++)
					{
						if (tb[k].usage < tb[k].capacity)
						{
							tmp_mdist = abs(x-tb[k].x) + abs(y-tb[k].y);
							if (criticality)
								tmp_mdist *= max(criticality[j], 0.01);

							if (tmp_mdist < mdist[j+1])
							{
								mdist[j+1] = tmp_mdist;
								nearest_tb[j] = k;
							}
						}
					}
					assert(nearest_tb[j] >= 0);
				}
				else
				{
					mdist[j+1] = FLT_MIN;
				}
			}
			heapsort(sort_idx, mdist, num_trace_nets, 0);
			sort_i = 1;
		}
	}

	for (i = 0; i < num_trace_nets; i++)
	{
		assert((*trace_iblk)[i] != OPEN);
		assert((*trace_ptc)[i] != OPEN);
	}

	free(mdist);
	free(sort_idx);
}

/** 
 * Main entry point for post-map tracing
 */
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
		float T_crit)
{
	boolean success;
	int old_num_nets;
	float *criticality;

	alloc_and_load_rr_node_route_structs();

	old_num_nets = num_nets;
	num_nets = new_num_nets;

	int itrace;
	int *trace_iblk, *trace_ptc;

	criticality = calloc(num_trace_nets, sizeof(float));
	/* Compute the criticality of each trace net for TIMING_DRIVEN,
	 * otherwise criticality = 0. */
	if (router_opts->inc_router_algorithm == TIMING_DRIVEN)
	{
		for (itrace = 0; itrace < num_trace_nets; itrace++)
		{
			int inet, num_sinks;
			inet = trace_nets[itrace];
			num_sinks = clb_net[inet].num_sinks;

			/* Adapted from timing_driven_route_net() */
			if (inet < old_num_nets)
			{
				int ipin;
				for (ipin = 1; ipin <= num_sinks; ipin++)
				{
					float pin_crit;
					pin_crit = max(router_opts->max_criticality - net_slack[inet][ipin] / T_crit, 0.);
					pin_crit = pow(pin_crit, router_opts->criticality_exp);
					pin_crit = min(pin_crit, router_opts->max_criticality);
					criticality[itrace] = max(criticality[itrace], pin_crit);
				}
			}
			else
			{
				int iblk, ptc;
				t_tnode *tnode;
				float pin_crit, slack;
				assert(num_sinks == 0);
				iblk = clb_net[inet].node_block[0];
				/* CLB OPIN ptc */
				ptc = clb_net[inet].node_block_pin[0];
				/* BLE OPIN ptc */
				ptc = block[iblk].pb->rr_graph[ptc].prev_node;
				assert(ptc != OPEN);
				tnode = block[iblk].pb->rr_graph[ptc].tnode;
				slack = tnode->T_req - tnode->T_arr;

				pin_crit = max(router_opts->max_criticality - slack / T_crit, 0.);
				pin_crit = pow(pin_crit, router_opts->criticality_exp);
				pin_crit = min(pin_crit, router_opts->max_criticality);
				criticality[itrace] = pin_crit;
			}
		}
	}

	/* Assign preferred trace-buffers to each trace net */
	inc_assign_trace_nets(trace_nets, num_trace_nets, tb, num_tb, criticality, &trace_iblk, &trace_ptc);

	/* Store the preferred targets into node_block[] and 
	 * net_rr_terminals[] */
	for (itrace = 0; itrace < num_trace_nets; itrace++)
	{
		int inet, num_sinks;
		int iblk, ptc, inode;
		inet = trace_nets[itrace];
		num_sinks = clb_net[inet].num_sinks;
		iblk = trace_iblk[itrace];
		ptc = trace_ptc[itrace];
		inode = get_rr_node_index(block[iblk].x, block[iblk].y, SINK, 
				ptc, rr_node_indices);
		assert(clb_net[inet].node_block[num_sinks+1] == OPEN);
		clb_net[inet].node_block[num_sinks+1] = iblk;
		assert(net_rr_terminals[inet][num_sinks+1] == OPEN);
		net_rr_terminals[inet][num_sinks+1] = inode;
	}

	free(trace_iblk);
	free(trace_ptc);

	/* If many-to-many flexibility is enabled, 
	 * mark all memories as targets */
	if (router_opts->inc_many_to_many == TRUE)
	{
		inc_mark_targets(num_tb, tb);
	}

	/* Start tracing */
	success = inc_try_trace(trace_nets,
			num_trace_nets,
			old_num_nets,
			router_opts, 
			FileNameOpts, 
			det_routing_arch, 
			timing_inf, 
			tb, 
			num_tb,
			clb_opins_used_locally,
			criticality);

	free_rr_node_route_structs();
	free(criticality);

	return success;
}
