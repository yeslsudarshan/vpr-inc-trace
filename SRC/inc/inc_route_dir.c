#include <stdio.h>
#include "vpr_types.h"
#include "globals.h"
#include "mst.h"
#include "route_common.h"
#include "route_directed_search.h"
#include "inc_trace.h"
#include "inc_route.h"
#include "inc_route_dir.h"
#include "assert.h"

/**
 * Adapted from get_directed_search_expected_cost()
 */
static float
inc_get_directed_search_expected_cost(int inode,
				  int target_x, int target_y,
				  float criticality_fac)
{

/* Determines the expected cost (due to resouce cost i.e. distance) to reach  *
 * the target node from inode.  It doesn't include the cost of inode --       *
 * that's already in the "known" path_cost.                                   */

    t_rr_type rr_type;
    int cost_index, ortho_cost_index, num_segs_same_dir, num_segs_ortho_dir;
    float cong_cost;

    rr_type = rr_node[inode].type;

    if(rr_type == CHANX || rr_type == CHANY)
	{
	    num_segs_same_dir =
		inc_get_expected_segs_to_target(inode, &num_segs_ortho_dir, target_x, target_y);
	    cost_index = rr_node[inode].cost_index;
	    ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;

	    cong_cost =
		num_segs_same_dir * rr_indexed_data[cost_index].base_cost +
		num_segs_ortho_dir *
		rr_indexed_data[ortho_cost_index].base_cost;
	    cong_cost +=
		rr_indexed_data[IPIN_COST_INDEX].base_cost +
		rr_indexed_data[SINK_COST_INDEX].base_cost;
	    return (1. - criticality_fac) * cong_cost;
	}

    else if(rr_type == IPIN)
	{			/* Change if you're allowing route-throughs */
	    return (rr_indexed_data[SINK_COST_INDEX].base_cost);
	}

    else
	{			/* Change this if you want to investigate route-throughs */
	    return (0.);
	}
}

/**
 * Adapted from directed_search_expand_trace_segment()
 */
static void
inc_directed_search_expand_trace_segment(struct s_trace *start_ptr,
				     float astar_fac,
					 int inet,
				     int remaining_connections_to_sink,
				     int target_x, int target_y,
					 float criticality_fac)
{

/* Adds all the rr_nodes in the entire traceback from SOURCE to all SINKS   *
 * routed so far (partial routing). 
 * This allows expansion to begin from the existing wiring.  The            *
 * remaining_connections_to_sink value is 0 if the route segment ending     *
 * at this location is the last one to connect to the SINK ending the route *
 * segment.  This is the usual case.  If it is not the last connection this *
 * net must make to this SINK, I have a hack to ensure the next connection  *
 * to this SINK goes through a different IPIN.  Without this hack, the      *
 * router would always put all the connections from this net to this SINK   *
 * through the same IPIN.  With LUTs or cluster-based logic blocks, you     *
 * should never have a net connecting to two logically-equivalent pins on   *
 * the same logic block, so the hack will never execute.  If your logic     *
 * block is an and-gate, however, nets might connect to two and-inputs on   *
 * the same logic block, and since the and-inputs are logically-equivalent, *
 * this means two connections to the same SINK.                             *
 *                                                                          *
 * For high-fanout nets, return the radius of the expansion bin,            *
 * undefined otherwise                                                      */

    struct s_trace *tptr;
    int inode, backward_path_cost, tot_cost;
	/*int target_x, target_y;*/
	/*int rlim, area;
	boolean success;*/

	/*
	target_x = rr_node[target_node].xhigh;
    target_y = rr_node[target_node].yhigh;
    	*/

#if 0
	area = (route_bb[inet].xmax - route_bb[inet].xmin) * (route_bb[inet].ymax - route_bb[inet].ymin);
	if(area <= 0) {
		area = 1;
	}
	
	if(clb_net[inet].num_sinks < HIGH_FANOUT_NET_LIM) {
		rlim = 1;
	} else {
		rlim = ceil(sqrt((float)area / (float)clb_net[inet].num_sinks));
		if(start_ptr == NULL) {
			/* For first node, route normally since there is nothing in the current traceback path */
			rlim = max(nx + 2, ny + 2);
		}
	}
	success = FALSE;
	/* determine quickly a feasible bin radius to route sink for high fanout nets 
	   this is necessary to prevent super long runtimes for high fanout nets; in best case, a reduction in complexity from O(N^2logN) to O(NlogN) (Swartz fast router)
	*/
	while(success == FALSE && start_ptr != NULL) {
		tptr = start_ptr;
		while(tptr != NULL && success == FALSE)
		{
			inode = tptr->index;
			if(!(rr_node[inode].type == IPIN || rr_node[inode].type == SINK)) {
				if( clb_net[inet].num_sinks < HIGH_FANOUT_NET_LIM ||
					(rr_node[inode].xlow <= target_x + rlim &&
					rr_node[inode].xhigh >= target_x - rlim &&
					rr_node[inode].ylow <= target_y + rlim &&
					rr_node[inode].yhigh >= target_y - rlim)) {
					success = TRUE;
				}
			}
			tptr = tptr->next;
		}
		
		if(success == FALSE) {
			if(rlim > max(nx + 2, ny + 2)) { 
				printf(ERRTAG "VPR internal error, net %s has paths that are not found in traceback\n", clb_net[inet].name);
				exit(1);
			}
			/* if sink not in bin, increase bin size until fit */
			rlim *= 2;
		} else {
			/* Sometimes might just catch a wire in the end segment, need to give it some channel space to explore */
			rlim += 4;
		}
	}
#endif

	if(remaining_connections_to_sink == 0)
	{			/* Usual case. */
		tptr = start_ptr;
		while(tptr != NULL)
		{
			/* WMF: partial routing is added to the heap with path cost of 0, because
			 * new extension to the next sink can start at any point on current partial 
			 * routing. However, for directed search the total cost must be made to favor
			 * the points of current partial routing that are NEAR the next sink (target sink) */

			/* WMF: IPINs and SINKs should be excluded from the heap in this
			 * since they NEVER connect TO any rr_node (no to_edges), but since they have
			 * no to_edges, it's ok (ROUTE_THROUGHS are disabled). To clarify, see 
			 * rr_graph.c to find out rr_node[inode].num_edges = 0 for SINKs and
			 * rr_node[inode].num_edges = 1 for INPINs */

			inode = tptr->index;
			if(!
			   (rr_node[inode].type == IPIN
			|| rr_node[inode].type == SINK))
			{
				/*
				if( clb_net[inet].num_sinks < HIGH_FANOUT_NET_LIM ||
					(rr_node[inode].xlow <= target_x + rlim &&
					rr_node[inode].xhigh >= target_x - rlim &&
					rr_node[inode].ylow <= target_y + rlim &&
					rr_node[inode].yhigh >= target_y - rlim)) */ {
					backward_path_cost = 0;
					tot_cost =
					backward_path_cost +
					astar_fac *
					inc_get_directed_search_expected_cost(inode, target_x, target_y, criticality_fac);
					node_to_heap(inode, tot_cost, NO_PREVIOUS,
						 NO_PREVIOUS, backward_path_cost,
						 OPEN);
				}
			}

			tptr = tptr->next;
		}
	}
	else
	{			/* This case never executes for most logic blocks. */
		printf("Warning: Multiple connections from net to the same sink. "
		   "This should not happen for LUT/Cluster based logic blocks. Aborting.\n");
		exit(1);
	}
	return /*rlim*/;
}

/**
 * Adapted from directed_search_add_source_to_heap()
 */
static void
inc_directed_search_add_inode_to_heap(int inode,
				   float astar_fac,
				   int target_x, int target_y,
				   float criticality_fac)
{

/* Adds the SOURCE of this net to the heap.  Used to start a net's routing. */

    /*int inode;*/
    float back_cost, tot_cost;

    /*inode = net_rr_terminals[inet][0];*/	/* SOURCE */
    back_cost = 0.0 + get_rr_cong_cost(inode);

    /* setting the total cost to 0 because it's the only element on the heap */
    /*
    if(!is_empty_heap())
	{
	    printf
		("Error: Wrong Assumption: in directed_search_add_source_to_heap "
		 "the heap is not empty. Need to properly calculate source node's cost.\n");
	    exit(1);
	}
    */

    /* WMF: path cost is 0. could use tot_cost = 0 to save some computation time, but
     * for consistency, I chose to do the expected cost calculation. */
    tot_cost =
		back_cost + astar_fac * inc_get_directed_search_expected_cost(inode, target_x, target_y, criticality_fac);

    node_to_heap(inode, tot_cost, NO_PREVIOUS, NO_PREVIOUS, back_cost, OPEN);
}

/**
 * Adapted from directed_search_expand_neighbours()
 */
static void
inc_directed_search_expand_neighbours(struct s_heap *current,
				  int inet, /* FIXME: not needed */
				  float bend_cost,
				  /*int target_node,*/
				  int target_x,
				  int target_y,
				  float astar_fac,
				  float criticality_fac)
{

/* Puts all the rr_nodes adjacent to current on the heap.  rr_nodes outside   *
 * the expanded bounding box specified in route_bb are not added to the     *
 * heap.  back_cost is the path_cost to get to inode. total cost i.e.
 * tot_cost is path_cost + (expected_cost to target sink) */

    int iconn, to_node, num_edges, inode/*, target_x, target_y*/;
    t_rr_type from_type, to_type;
    float new_tot_cost, old_back_pcost, new_back_pcost;

    inode = current->index;
    old_back_pcost = current->backward_path_cost;
    num_edges = rr_node[inode].num_edges;

    /*
    target_x = rr_node[target_node].xhigh;
    target_y = rr_node[target_node].yhigh;
    */

    for(iconn = 0; iconn < num_edges; iconn++)
	{
	    to_node = rr_node[inode].edges[iconn];

	    /*
	    if(rr_node[to_node].xhigh < route_bb[inet].xmin ||
	       rr_node[to_node].xlow > route_bb[inet].xmax ||
	       rr_node[to_node].yhigh < route_bb[inet].ymin ||
	       rr_node[to_node].ylow > route_bb[inet].ymax)
		continue;*/	/* Node is outside (expanded) bounding box. */

	    /*
		if(clb_net[inet].num_sinks >= HIGH_FANOUT_NET_LIM) {
			if(rr_node[to_node].xhigh < target_x - highfanout_rlim ||
				rr_node[to_node].xlow > target_x + highfanout_rlim ||
				rr_node[to_node].yhigh < target_y - highfanout_rlim ||
				rr_node[to_node].ylow > target_y + highfanout_rlim)
			continue;*/	/* Node is outside high fanout bin. */
		/*}*/

/* Prune away IPINs that lead to blocks other than the target one.  Avoids  *
 * the issue of how to cost them properly so they don't get expanded before *
 * more promising routes, but makes route-throughs (via CLBs) impossible.   *
 * Change this if you want to investigate route-throughs.                   */

	    /*
	    to_type = rr_node[to_node].type;
	    if(to_type == IPIN && (rr_node[to_node].xhigh != target_x ||
				   rr_node[to_node].yhigh != target_y))
		continue;
	    */

		/* EH: Ignore nodes that are already fully occupied by
		 * the user circuit */
	    if((rr_node[to_node].occ-rr_node[to_node].inc_occ) >= rr_node[to_node].capacity)
	    	continue;

/* new_back_pcost stores the "known" part of the cost to this node -- the   *
 * congestion cost of all the routing resources back to the existing route  *
 * new_tot_cost 
 * is this "known" backward cost + an expected cost to get to the target.   */

	    new_back_pcost = old_back_pcost + (1. - criticality_fac) * get_rr_cong_cost(to_node);

	    if(bend_cost != 0.)
		{
		    from_type = rr_node[inode].type;
		    to_type = rr_node[to_node].type;
		    if((from_type == CHANX && to_type == CHANY) ||
		       (from_type == CHANY && to_type == CHANX))
			new_back_pcost += bend_cost;
		}

	    /* Calculate the new total cost = path cost + astar_fac * remaining distance to target */
	    new_tot_cost = new_back_pcost + astar_fac *
		 (to_node, target_x, target_y, criticality_fac);

	    node_to_heap(to_node, new_tot_cost, inode, iconn, new_back_pcost,
			 OPEN);
	}
}

/**
 * Adapted from directed_search_route_net()
 */
boolean
inc_directed_search_route_net(int inet,
			  float pres_fac,
			  float astar_fac,
			  float bend_cost,
			  struct s_trace **old_trace_tail,
			  int old_num_nets,
			  struct s_router_opts *router_opts,
			  int num_tb,
			  t_trace_buffer *tb,
			  float target_criticality)
{

/* Uses a maze routing (Dijkstra's) algorithm to route a net.  The net       *
 * begins at the net output, and expands outward until it hits a target      *
 * pin.  The algorithm is then restarted with the entire first wire segment  *
 * included as part of the source this time.  For an n-pin net, the maze     *
 * router is invoked n-1 times to complete all the connections.  Inet is     *
 * the index of the net to be routed.  Bends are penalized by bend_cost      *
 * (which is typically zero for detailed routing and nonzero only for global *
 * routing), since global routes with lots of bends are tougher to detailed  *
 * route (using a detailed router like SEGA).                                *
 * If this routine finds that a net *cannot* be connected (due to a complete *
 * lack of potential paths, rather than congestion), it returns FALSE, as    *
 * routing is impossible on this architecture.  Otherwise it returns TRUE.   */
/* WMF: This is the directed search (A-star) version of maze router. */

    int inode, ipin, remaining_connections_to_sink;
    int itarget, target_inode, target_iblk;
    struct s_heap *current;
    struct s_trace *new_route_start_tptr;
    float old_tcost, new_tcost, old_back_cost, new_back_cost;
	int target_x, target_y;
	/*int highfanout_rlim;*/

/* Rip-up any old routing. */
    /* WMF: For the 1st router iteration trace_head[inet] is NULL, as it is 
     * my_calloc'ed in alloc_route_structs() so the following does nothing.
     * However, for subsequent iterations, trace_head[inet] contains the previous
     * ieration's routing for this net. */
    /*
    pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
    free_traceback(inet);*/	/* kills trace, and set the trace head and tail to NULL */

	/* Rip up incremental trace only */
    inc_free_traceback(inet, old_trace_tail);

    /* adding the SOURCE node to the heap with correct total cost */
    /*
    target_pin = mst[inet][0].to_node;
    target_node = net_rr_terminals[inet][target_pin];
    */
	inode = net_rr_terminals[inet][0];
	
	/* EH: Extract the assigned TB target */
	itarget = clb_net[inet].num_sinks;
	target_inode = net_rr_terminals[inet][itarget+1];
	assert(target_inode != OPEN);
	target_iblk = clb_net[inet].node_block[itarget+1];
	target_x = rr_node[target_inode].xlow;
	target_y = rr_node[target_inode].ylow;

	/* If many-to-many flexibility is disabled, then mark
	 * its one and only pre-assigned target */
	if (!router_opts->inc_many_to_many)
	{
		assert(rr_node_route_inf[target_inode].target_flag == 0);
		rr_node_route_inf[target_inode].target_flag = target_iblk;
	}
	/* If LE symmetry enabled, and inet was local, add all CLB_OPINs 
	 * from other local nets onto heap too */
	if (router_opts->inc_le_symmetry && inet >= old_num_nets)
	{
		int num_sources, *sources;
		int i;
		boolean found = FALSE;
		num_sources = inc_local_OPINs(inet, old_num_nets, &sources);
		assert(num_sources > 0);
		for (i = 0; i < num_sources; i++)
		{
			inc_directed_search_add_inode_to_heap(sources[i], astar_fac, target_x, target_y, target_criticality);
			if (sources[i] == inode)
				found = TRUE;
		}
		assert(found == TRUE);
		free(sources);
	}
	else
	{
		inc_directed_search_add_inode_to_heap(inode, astar_fac, target_x, target_y, target_criticality);
	}
    /*mark_ends(inet);*/

    remaining_connections_to_sink = 0;

	/*
    itarget = 0;
    for(itarget = 0; itarget < clb_net[inet].num_sinks; itarget++)*/
	{
	    /*
	    target_pin = mst[inet][itarget].to_node;
	    target_node = net_rr_terminals[inet][target_pin];
	    */

	    /*    printf ("Target #%d, pin number %d, target_node: %d.\n",
	     * itarget, target_pin, target_node);  */

	    /* WMF: since the heap has been emptied, need to restart the wavefront
	     * from the current partial routing, starting at the trace_head (SOURCE) 
	     * Note: in the 1st iteration, there is no trace (no routing at all for this net)
	     * i.e. trace_head[inet] == NULL (found in free_traceback() in route_common.c, 
	     * which is called before the routing of any net), 
	     * so this routine does nothing, but the heap does have the SOURCE node due 
	     * to directed_search_add_source_to_heap (inet) before the loop */
	    /*highfanout_rlim = */inc_directed_search_expand_trace_segment(trace_head[inet],
						 astar_fac, inet,
						 remaining_connections_to_sink,
						 target_x, target_y,
						 target_criticality);

	    current = get_heap_head();

	    if(current == NULL)
		{		/* Infeasible routing.  No possible path for net. */
			if (!router_opts->inc_many_to_many)
			{
	    		rr_node_route_inf[target_inode].target_flag = 0;
			}
		    reset_path_costs();	/* Clean up before leaving. */
		    return (FALSE);
		}

	    inode = current->index;

	    /*while(inode != target_node)*/
	    while(rr_node_route_inf[inode].target_flag != target_iblk)
		{
		    old_tcost = rr_node_route_inf[inode].path_cost;
		    new_tcost = current->cost;

		    /* WMF: not needed if Vaughn initialized rr_node_route_inf[inode].backward_path_cost
		     * to HUGE_FLOAT along with rr_node_route_inf[inode].path_cost */
		    if(old_tcost > 0.99 * HUGE_FLOAT)	/* First time touched. */
			old_back_cost = HUGE_FLOAT;
		    else
			old_back_cost =
			    rr_node_route_inf[inode].backward_path_cost;

		    new_back_cost = current->backward_path_cost;

		    /* I only re-expand a node if both the "known" backward cost is lower  *
		     * in the new expansion (this is necessary to prevent loops from       *
		     * forming in the routing and causing havoc) *and* the expected total  *
		     * cost to the sink is lower than the old value.  Different R_upstream *
		     * values could make a path with lower back_path_cost less desirable   *
		     * than one with higher cost.  Test whether or not I should disallow   *
		     * re-expansion based on a higher total cost.                          */

		    /* updating the maze (Dijktra's min dist algorithm) if found "shorter" path */
		    if(old_tcost > new_tcost && old_back_cost > new_back_cost)
/*       if (old_tcost > new_tcost)      */
			{
			    rr_node_route_inf[inode].prev_node =
				current->u.prev_node;
			    rr_node_route_inf[inode].prev_edge =
				current->prev_edge;
			    rr_node_route_inf[inode].path_cost = new_tcost;
			    rr_node_route_inf[inode].backward_path_cost =
				new_back_cost;

			    if(old_tcost > 0.99 * HUGE_FLOAT)	/* First time touched. */
				add_to_mod_list(&rr_node_route_inf[inode].
						path_cost);

			    inc_directed_search_expand_neighbours(current, inet,
							      bend_cost,
								  target_x, target_y,
							      astar_fac,
								  target_criticality);
			}

		    free_heap_data(current);
		    current = get_heap_head();

		    if(current == NULL)
			{	/* Impossible routing.  No path for net. */
				printf("Failed to route net %s #%d pin %d num_sinks %d\n", clb_net[inet].name, inet, itarget, clb_net[inet].num_sinks);
				if (!router_opts->inc_many_to_many)
				{
		    		rr_node_route_inf[target_inode].target_flag = 0;
				}
			    reset_path_costs();
			    return (FALSE);
			}

		    inode = current->index;
		}

		/* EH: If many-to-many was disabled, check target was 
		 * reached and unmark it */
		if (!router_opts->inc_many_to_many)
		{
			assert(target_inode == inode);
	    	/*rr_node_route_inf[inode].target_flag--;*/	/* Connected to this SINK. */
	    	rr_node_route_inf[inode].target_flag = 0;
		}

	    remaining_connections_to_sink =
		rr_node_route_inf[inode].target_flag;

	    /* keep info on the current routing of this net */
	    new_route_start_tptr = update_traceback(current, inet);

	    free_heap_data(current);
	    /* update the congestion costs of rr_nodes due to the routing to this sink 
	     * so only those nodes used in the partial routing of this sink and not 
	     * of the entire net (remember we're in a loop for this net over its sinks) */
	    /*pathfinder_update_one_cost(new_route_start_tptr, 1, pres_fac);*/

	    /* WMF: MUST empty heap and recalculate all total costs, because
	     * for the next sink, the target destination is changed, so the expected
	     * cost calculation is changed also, meaning all the nodes on the heap have
	     * "stale" total costs (costs based on last sink). */
	    empty_heap();
	    reset_path_costs();
	}

    return (TRUE);
}
