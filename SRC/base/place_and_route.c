#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include "util.h"
#include "vpr_types.h"
#include "vpr_utils.h"
#include "globals.h"
#include "place_and_route.h"
#include "mst.h"
#include "place.h"
#include "read_place.h"
#include "route_export.h"
#include "draw.h"
#include "stats.h"
#include "check_route.h"
#include "rr_graph.h"
#include "path_delay.h"
#include "net_delay.h"
#include "timing_place.h"
#include "read_xml_arch_file.h"
#include "inc_trace.h"
#include "inc_misc.h"
#include "route_export.h"
#include <float.h>
#include <signal.h>

/******************* Subroutines local to this module ************************/

static int binary_search_place_and_route(struct s_placer_opts placer_opts,
					 struct s_file_name_opts *FileNameOpts,
					 boolean full_stats,
					 boolean verify_binary_search,
					 struct s_annealing_sched
					 annealing_sched,
					 struct s_router_opts router_opts,
					 struct s_det_routing_arch
					 det_routing_arch,
					 t_segment_inf * segment_inf,
					 t_timing_inf timing_inf,
					 t_chan_width_dist chan_width_dist,
					 t_mst_edge ** mst,
					 t_model *models);

static float comp_width(t_chan * chan,
			float x,
			float separation);


void post_place_sync(INP int num_blocks,
		     INOUTP const struct s_block block_list[]);

void free_pb_data(t_pb *pb);

/************************* Subroutine Definitions ****************************/

void
place_and_route(enum e_operation operation,
		struct s_placer_opts placer_opts,
		struct s_file_name_opts *FileNameOpts,
		struct s_annealing_sched annealing_sched,
		struct s_router_opts router_opts,
		struct s_det_routing_arch det_routing_arch,
		t_segment_inf * segment_inf,
		t_timing_inf timing_inf,
		t_chan_width_dist chan_width_dist,
		struct s_model *models,
		char *trace_string,
		t_arch *Arch)
{

/* This routine controls the overall placement and routing of a circuit. */
    char msg[BUFSIZE];
    int width_fac, inet, i;
    boolean success, Fc_clipped;
    float **net_delay, **net_slack;
    struct s_linked_vptr *net_delay_chunk_list_head;
    t_ivec **clb_opins_used_locally;	/* [0..num_blocks-1][0..num_class-1] */
    t_mst_edge **mst = NULL;	/* Make sure mst is never undefined */
    int max_pins_per_clb;
    clock_t begin, end;

    int *trace_nets, num_trace_nets, new_num_nets, old_num_nets;

	int num_tb;
	t_trace_buffer *tb;

	Fc_clipped = FALSE;

    max_pins_per_clb = 0;
    for(i = 0; i < num_types; i++)
	{
	    if(type_descriptors[i].num_pins > max_pins_per_clb)
		{
		    max_pins_per_clb = type_descriptors[i].num_pins;
		}
	}

    begin = clock();
    if(placer_opts.place_freq == PLACE_NEVER)
	{
	    /* Read the placement from a file */
	    read_place(FileNameOpts->PlaceFile, FileNameOpts->NetFile, FileNameOpts->ArchFile, nx, ny, num_blocks,
		       block);
	    sync_grid_to_blocks(num_blocks, block, nx, ny, grid);
	}
    else
	{
	    assert((PLACE_ONCE == placer_opts.place_freq) ||
		   (PLACE_ALWAYS == placer_opts.place_freq));
	    try_place(placer_opts, annealing_sched, chan_width_dist,
		      router_opts, det_routing_arch, segment_inf,
		      timing_inf, &mst);
	    print_place(FileNameOpts->PlaceFile, FileNameOpts->NetFile, FileNameOpts->ArchFile);

	}
    end = clock();
#ifdef CLOCKS_PER_SEC
    printf("Placement took %g seconds\n", (float)(end - begin) / CLOCKS_PER_SEC);
#else
    printf("Placement took %g seconds\n", (float)(end - begin) / CLK_PER_SEC);
#endif

    begin = clock();
    post_place_sync(num_blocks, block);


    fflush(stdout);

	/* reset mst */
	if(mst)
	{
	    for(inet = 0; inet < num_nets; inet++)
		{
			if(mst[inet]) {
				free(mst[inet]);
			}
		}
	    free(mst);
	}
	mst = NULL;

	if(!router_opts.doRouting)
		return;

	/* EDDIE: If we're doing any incremental routing */
    if (router_opts.inc_type != INC_NONE)
    {
	    /* Infer each vpack_net with node_block[] and 
		 * node_block_pins[] from pb structure */
		inc_infer_vpack_blocks_and_pins();

		/* Place trace-buffers into all spare memory locations */
		tb = NULL;
		num_tb = inc_reclaim_tbs(&tb, Arch);

		/* For mid-map insertion only */
	    if (router_opts.inc_type == INC_MIDMAP)
	    {
			int *trace_iblk, *trace_ptc;
			int iblk, ptc;

			/* Parse the trace string */
		    num_trace_nets = inc_parse_trace_string(trace_string, &trace_nets, &new_num_nets);

			/* Iteratively assign each trace net furthest away from 
			 * any remaining trace-buffer to the nearest trace-buffer */
		    fprintf(stdout, "Instrumenting %d nets prior to routing...\n", num_trace_nets);
			fprintf(stdout, "User num_nets = %d\n", num_nets);
			num_nets = new_num_nets;
		    inc_assign_trace_nets(trace_nets, num_trace_nets, tb, num_tb, NULL, &trace_iblk, &trace_ptc);

			/* Now connect them up by adding those pins as extra sinks */
			for (i = 0; i < num_trace_nets; i++)
			{
				inet = trace_nets[i];
				iblk = trace_iblk[i];
				ptc = trace_ptc[i];
				inc_connect_net(inet, iblk, ptc);
			}

			fprintf(stdout, "Instrumented num_nets = %d\n\n", num_nets);
			free(trace_iblk);
			free(trace_ptc);
		    free(trace_nets);
	    }
    }

    mst = (t_mst_edge **) my_malloc(sizeof(t_mst_edge *) * num_nets);
    for(inet = 0; inet < num_nets; inet++)
	{
	    mst[inet] = get_mst_of_net(inet);
	}

    width_fac = router_opts.fixed_channel_width;


    /* If channel width not fixed, use binary search to find min W */
    if(NO_FIXED_CHANNEL_WIDTH == width_fac)
	{
	    binary_search_place_and_route(placer_opts, FileNameOpts,
					  router_opts.full_stats, router_opts.verify_binary_search,
					  annealing_sched, router_opts,
					  det_routing_arch, segment_inf,
					  timing_inf,
					  chan_width_dist, mst, models);
	}
    else
	{
	    if(det_routing_arch.directionality == UNI_DIRECTIONAL)
		{
		    if(width_fac % 2 != 0)
			{
			    printf
				("Error: pack_place_and_route.c: given odd chan width (%d) for udsd architecture\n",
				 width_fac);
			    exit(1);
			}
		}
	    /* Other constraints can be left to rr_graph to check since this is one pass routing */

	    /* Allocate the major routing structures. */

	    clb_opins_used_locally = alloc_route_structs();

	    if(timing_inf.timing_analysis_enabled)
		{
		    net_slack =
			alloc_and_load_timing_graph(timing_inf);
		    net_delay = alloc_net_delay(&net_delay_chunk_list_head, clb_net, num_nets);
		}
	    else
		{
		    net_delay = NULL;	/* Defensive coding. */
		    net_slack = NULL;
		}

	    success =
		try_route(width_fac, router_opts, det_routing_arch,
			  segment_inf, timing_inf, net_slack, net_delay,
			  chan_width_dist, clb_opins_used_locally, mst,
			  &Fc_clipped, operation, FileNameOpts);

	    if(Fc_clipped)
		{
		    printf
			("Warning: Fc_output was too high and was clipped to full (maximum) connectivity.\n");
		}

		/* EDDIE: For post-map insertion only */
    	if (router_opts.inc_type == INC_POSTMAP)
    	{
			/* If normal user circuit routing was successful */
			if (success)
			{
				int inode;
				float T_crit;

				/* Now load the post-routing timing slack */
			    load_net_delay_from_routing(net_delay, clb_net, num_nets);
			    load_timing_graph_net_delays(net_delay);
		    	T_crit = load_net_slack(net_slack, 0);

				/* Parse the trace_string */
	        	num_trace_nets = inc_parse_trace_string(trace_string, &trace_nets, &new_num_nets);
				old_num_nets = num_nets;

				/* And try to do some incremental tracing */
				success = inc_trace(trace_nets,
    			    		num_trace_nets,
    			    		new_num_nets,
							tb,
							num_tb,
    			    		&router_opts,
    			    		det_routing_arch,
    			    		timing_inf,
    			      		FileNameOpts,
    			    		clb_opins_used_locally,
							net_slack,
							T_crit);

				/* The RRG has changed, re-create timing graph */
				if(success && timing_inf.timing_analysis_enabled)
				{
				    assert(net_slack);
				    free_timing_graph(net_slack);
				    assert(net_delay);
				    free_net_delay(net_delay, &net_delay_chunk_list_head);

				    net_slack = alloc_and_load_timing_graph(timing_inf);
				    net_delay = alloc_net_delay(&net_delay_chunk_list_head, clb_net, num_nets);
				}

				free(trace_nets);
			}
		}
		
	    if(success == FALSE)
		{
		    printf
			("Circuit is unrouteable with a channel width factor of %d\n\n",
			 width_fac);
		    sprintf(msg,
			    "Routing failed with a channel width factor of %d.  ILLEGAL routing shown.",
			    width_fac);
		}

	    else
		{
		    printf("\n");
		    check_route(router_opts.route_type,
				det_routing_arch.num_switch,
				clb_opins_used_locally);
		    printf("\n");
		    get_serial_num();

		    printf
			("Circuit successfully routed with a channel width factor of %d.\n\n",
			 width_fac);

		    routing_stats(router_opts.full_stats, router_opts.route_type,
				  det_routing_arch.num_switch, segment_inf,
				  det_routing_arch.num_segment,
				  det_routing_arch.R_minW_nmos,
				  det_routing_arch.R_minW_pmos,
				  det_routing_arch.directionality,
				  timing_inf.timing_analysis_enabled,
				  net_slack, net_delay);

#ifdef PRINT_TIMING
		    print_net_delay(net_delay, "before.net_delay", clb_net, num_nets);
		    print_net_slack("before.net_slack", net_slack);
		    print_critical_path("before.critical_path");
		    print_critical_path_dot("before.critical_path.dot");
		    print_timing_graph("before.timing_graph");
#endif

			/* If we previously read the routing, then don't overwrite it,
			 * instead, output the instrumented ruote into <circuitname>_inc.route */
    		if (router_opts.inc_type != INC_NONE)
			{
				char *route_file;
				route_file = malloc((strlen(FileNameOpts->CircuitName)+strlen("_inc.route")+1)*sizeof(char));
				sprintf(route_file, "%s_inc.route", FileNameOpts->CircuitName);
				print_route(route_file);
				free(route_file);
			}
			else
			{
		    	print_route(FileNameOpts->RouteFile);
			}

#ifdef CREATE_ECHO_FILES
		    /*print_sink_delays("routing_sink_delays.echo"); */
#endif /* CREATE_ECHO_FILES */

		    sprintf(msg,
			    "Routing succeeded with a channel width factor of %d.\n\n",
			    width_fac);

		}

	    init_draw_coords(max_pins_per_clb);
	    update_screen(MAJOR, msg, ROUTING,
			  timing_inf.timing_analysis_enabled);

	    if(timing_inf.timing_analysis_enabled)
		{
		    assert(net_slack);
			#ifdef CREATE_ECHO_FILES
				print_timing_graph_as_blif("post_flow_timing_graph.blif", models);
			#endif

		    free_timing_graph(net_slack);

		    assert(net_delay);
		    free_net_delay(net_delay, &net_delay_chunk_list_head);
		}

	    free_route_structs(clb_opins_used_locally);

		/* To prevent errors when cleaning up below,
		 * revert num_nets back to its old value */
   		if (router_opts.inc_type == INC_POSTMAP)
		{
			num_nets = old_num_nets;
		}

	    fflush(stdout);

	}
	end = clock();
	#ifdef CLOCKS_PER_SEC
		printf("Routing took %g seconds\n", (float)(end - begin) / CLOCKS_PER_SEC);
	#else
		printf("Routing took %g seconds\n", (float)(end - begin) / CLK_PER_SEC);
	#endif

	if (router_opts.inc_type != INC_NONE)
	{
		free(tb);
	}

    /*WMF: cleaning up memory usage */
    if(mst)
	{
	    for(inet = 0; inet < num_nets; inet++)
		{
			if(!mst[inet]) {
				printf("no mst for net %s #%d\n", clb_net[inet].name, inet);
			}
		    assert(mst[inet]);
		    free(mst[inet]);
		}
	    free(mst);
	    mst = NULL;
	}
}



static int
binary_search_place_and_route(struct s_placer_opts placer_opts,
			      struct s_file_name_opts *FileNameOpts,
			      boolean full_stats,
			      boolean verify_binary_search,
			      struct s_annealing_sched annealing_sched,
			      struct s_router_opts router_opts,
			      struct s_det_routing_arch det_routing_arch,
			      t_segment_inf * segment_inf,
			      t_timing_inf timing_inf,
			      t_chan_width_dist chan_width_dist,
			      t_mst_edge ** mst,
			          t_model *models)
{

/* This routine performs a binary search to find the minimum number of      *
 * tracks per channel required to successfully route a circuit, and returns *
 * that minimum width_fac.                                                  */

    struct s_trace **best_routing;	/* Saves the best routing found so far. */
    int current, low, high, final;
    int max_pins_per_clb, i;
    boolean success, prev_success, prev2_success, Fc_clipped = FALSE;
    char msg[BUFSIZE];
    float **net_delay, **net_slack;
    struct s_linked_vptr *net_delay_chunk_list_head;
    int try_w_limit;
    t_ivec **clb_opins_used_locally, **saved_clb_opins_used_locally;

    /* [0..num_blocks-1][0..num_class-1] */
    int attempt_count;
    int udsd_multiplier;
    int warnings;

	t_graph_type graph_type;

/* Allocate the major routing structures. */

	if(router_opts.route_type == GLOBAL) {
		graph_type = GRAPH_GLOBAL;
	} else {
		graph_type = (det_routing_arch.directionality ==
		    BI_DIRECTIONAL ? GRAPH_BIDIR : GRAPH_UNIDIR);
	}

    max_pins_per_clb = 0;
    for(i = 0; i < num_types; i++)
	{
	    max_pins_per_clb =
		max(max_pins_per_clb, type_descriptors[i].num_pins);
	}

    clb_opins_used_locally = alloc_route_structs();
    best_routing = alloc_saved_routing(clb_opins_used_locally,
				       &saved_clb_opins_used_locally);

    if(timing_inf.timing_analysis_enabled)
	{
	    net_slack =
		alloc_and_load_timing_graph(timing_inf);
	    net_delay = alloc_net_delay(&net_delay_chunk_list_head, clb_net, num_nets);
	}
    else
	{
	    net_delay = NULL;	/* Defensive coding. */
	    net_slack = NULL;
	}

    /* UDSD by AY Start */
    if(det_routing_arch.directionality == BI_DIRECTIONAL)
	udsd_multiplier = 1;
    else
	udsd_multiplier = 2;
    /* UDSD by AY End */

    if(router_opts.fixed_channel_width != NO_FIXED_CHANNEL_WIDTH)
	{
	    current = router_opts.fixed_channel_width + 5 * udsd_multiplier;
	    low = router_opts.fixed_channel_width - 1 * udsd_multiplier;
	}
    else
	{
	    current = max_pins_per_clb + max_pins_per_clb % 2;	/* Binary search part */
	    low = -1;
	}

    /* Constraints must be checked to not break rr_graph generator */
    if(det_routing_arch.directionality == UNI_DIRECTIONAL)
	{
	    if(current % 2 != 0)
		{
		    printf
			("Error: pack_place_and_route.c: tried odd chan width (%d) for udsd architecture\n",
			 current);
		    exit(1);
		}
	}

    else
	{
	    if(det_routing_arch.Fs % 3)
		{
		    printf("Fs must be three in bidirectional mode\n");
		    exit(1);
		}
	}

    high = -1;
    final = -1;
    try_w_limit = 0;

    attempt_count = 0;

    while(final == -1)
	{
	    printf("low, high, current %d %d %d\n", low, high, current);
	    fflush(stdout);

/* Check if the channel width is huge to avoid overflow.  Assume the *
 * circuit is unroutable with the current router options if we're    *
 * going to overflow.                                                */
	    if(router_opts.fixed_channel_width != NO_FIXED_CHANNEL_WIDTH)
		{
		    if(current > router_opts.fixed_channel_width * 4)
			{
			    printf
				("This circuit appears to be unroutable with the current "
				 "router options. Last failed at %d\n", low);
			    printf("Aborting routing procedure.\n");
			    exit(1);
			}
		}
	    else
		{
		    if(current > 1000)
			{
			    printf
				("This circuit requires a channel width above 1000, probably isn't going to route.\n");
			    printf("Aborting routing procedure.\n");
			    exit(1);
			}
		}

	    if((current * 3) < det_routing_arch.Fs)
		{
		    printf
			("width factor is now below specified Fs. Stop search.\n");
		    final = high;
		    break;
		}

	    if(placer_opts.place_freq == PLACE_ALWAYS)
		{
		    placer_opts.place_chan_width = current;
		    try_place(placer_opts, annealing_sched, chan_width_dist,
			      router_opts, det_routing_arch, segment_inf,
			      timing_inf, &mst);
		}

	    free_rr_node_route_structs();
	    success =
		try_route(current, router_opts, det_routing_arch, segment_inf,
			  timing_inf, net_slack, net_delay, chan_width_dist,
			  clb_opins_used_locally, mst, &Fc_clipped, RUN_FLOW, FileNameOpts);
	    attempt_count++;
	    fflush(stdout);
#if 1
	    if(success && (Fc_clipped == FALSE))
#else
	    if(success
	       && (Fc_clipped == FALSE
		   || det_routing_arch.Fc_type == FRACTIONAL))
#endif
		{
			if(current == high) {
				/* Can't go any lower */
				final = current;
			}
			high = current;

		    /* If Fc_output is too high, set to full connectivity but warn the user */
		    if(Fc_clipped)
			{
			    printf
				("Warning: Fc_output was too high and was clipped to full (maximum) connectivity.\n");
			}

/* If we're re-placing constantly, save placement in case it is best. */
#if 0
		    if(placer_opts.place_freq == PLACE_ALWAYS)
			{
			    print_place(place_file, net_file, arch_file);
			}
#endif

		    /* Save routing in case it is best. */
		    save_routing(best_routing, clb_opins_used_locally,
				 saved_clb_opins_used_locally);

		    if((high - low) <= 1 * udsd_multiplier)
			final = high;

		    if(low != -1)
			{
			    current = (high + low) / 2;
			}
		    else
			{
			    current = high / 2;	/* haven't found lower bound yet */
			}
		}
	    else
		{		/* last route not successful */
		    if(success && Fc_clipped)
			{
			    printf
				("Routing rejected, Fc_output was too high.\n");
			    success = FALSE;
			}
		    low = current;
		    if(high != -1)
			{

			    if((high - low) <= 1 * udsd_multiplier)
				final = high;

			    current = (high + low) / 2;
			}
		    else
			{
			    if(router_opts.fixed_channel_width !=
			       NO_FIXED_CHANNEL_WIDTH)
				{
				    /* FOR Wneed = f(Fs) search */
				    if(low <
				       router_opts.fixed_channel_width + 30)
					{
					    current =
						low + 5 * udsd_multiplier;
					}
				    else
					{
					    printf
						("Aborting: Wneed = f(Fs) search found exceedingly large Wneed (at least %d)\n",
						 low);
					    exit(1);
					}
				}
			    else
				{
				    current = low * 2;	/* Haven't found upper bound yet */
				}
			}
		}
	    current = current + current % udsd_multiplier;
	}

/* The binary search above occassionally does not find the minimum    *
 * routeable channel width.  Sometimes a circuit that will not route  *
 * in 19 channels will route in 18, due to router flukiness.  If      *  
 * verify_binary_search is set, the code below will ensure that FPGAs *
 * with channel widths of final-2 and final-3 wil not route           *  
 * successfully.  If one does route successfully, the router keeps    *
 * trying smaller channel widths until two in a row (e.g. 8 and 9)    *
 * fail.                                                              */

    if(verify_binary_search)
	{

	    printf
		("\nVerifying that binary search found min. channel width ...\n");

	    prev_success = TRUE;	/* Actually final - 1 failed, but this makes router */
	    /* try final-2 and final-3 even if both fail: safer */
	    prev2_success = TRUE;

	    current = final - 2;

	    while(prev2_success || prev_success)
		{
		    if((router_opts.fixed_channel_width !=
			NO_FIXED_CHANNEL_WIDTH)
		       && (current < router_opts.fixed_channel_width))
			{
			    break;
			}
		    fflush(stdout);
		    if(current < 1)
			break;
		    if(placer_opts.place_freq == PLACE_ALWAYS)
			{
			    placer_opts.place_chan_width = current;
			    try_place(placer_opts, annealing_sched,
				      chan_width_dist, router_opts,
				      det_routing_arch, segment_inf,
				      timing_inf, &mst);
			}

		    success =
			try_route(current, router_opts, det_routing_arch,
				  segment_inf, timing_inf, net_slack,
				  net_delay, chan_width_dist,
				  clb_opins_used_locally, mst, &Fc_clipped, RUN_FLOW, NULL);

		    if(success && Fc_clipped == FALSE)
			{
			    final = current;
			    save_routing(best_routing, clb_opins_used_locally,
					 saved_clb_opins_used_locally);

			    if(placer_opts.place_freq == PLACE_ALWAYS)
				{
				    print_place(FileNameOpts->PlaceFile, FileNameOpts->NetFile,
						FileNameOpts->ArchFile);
				}
			}


		    prev2_success = prev_success;
		    prev_success = success;
		    current--;
			if(det_routing_arch.directionality == UNI_DIRECTIONAL) {
				current--; /* width must be even */
			}
		}
	}



    /* End binary search verification. */
    /* Restore the best placement (if necessary), the best routing, and  *
     * * the best channel widths for final drawing and statistics output.  */
    init_chan(final, chan_width_dist);
#if 0
    if(placer_opts.place_freq == PLACE_ALWAYS)
	{
	    printf("Reading best placement back in.\n");
	    placer_opts.place_chan_width = final;
	    read_place(place_file, net_file, arch_file, placer_opts,
		       router_opts, chan_width_dist, det_routing_arch,
		       segment_inf, timing_inf, &mst);
	}
#endif
    free_rr_graph();

    build_rr_graph(graph_type,
		   num_types, type_descriptors, nx, ny, grid,
		   chan_width_x[0], NULL,
		   det_routing_arch.switch_block_type, det_routing_arch.Fs,
		   det_routing_arch.num_segment, det_routing_arch.num_switch, segment_inf,
		   det_routing_arch.global_route_switch,
		   det_routing_arch.delayless_switch, timing_inf,
		   det_routing_arch.wire_to_ipin_switch,
		   router_opts.base_cost_type, &warnings);

    restore_routing(best_routing, clb_opins_used_locally,
		    saved_clb_opins_used_locally);

    printf("\n");
    check_route(router_opts.route_type, det_routing_arch.num_switch,
		clb_opins_used_locally);
    printf("\n");
    get_serial_num();
    if(Fc_clipped)
	{
	    printf
		("Warning: Best routing Fc_output too high, clipped to full (maximum) connectivity.\n");
	}
    printf("Best routing used a channel width factor of %d.\n\n", final);

    routing_stats(full_stats, router_opts.route_type,
		  det_routing_arch.num_switch, segment_inf,
		  det_routing_arch.num_segment,
		  det_routing_arch.R_minW_nmos,
		  det_routing_arch.R_minW_pmos,
		  det_routing_arch.directionality,
		  timing_inf.timing_analysis_enabled, net_slack, net_delay);
#ifdef PRINT_TIMING
    print_net_delay(net_delay, "before.net_delay", clb_net, num_nets);
    print_net_slack("before.net_slack", net_slack);
    print_critical_path("before.critical_path");
    print_critical_path_dot("before.critical_path.dot");
    print_timing_graph("before.timing_graph");
#endif


    print_route(FileNameOpts->RouteFile);

#ifdef CREATE_ECHO_FILES
    /* print_sink_delays("routing_sink_delays.echo"); */
#endif /* CREATE_ECHO_FILES */

    init_draw_coords(max_pins_per_clb);
    sprintf(msg, "Routing succeeded with a channel width factor of %d.",
	    final);
    update_screen(MAJOR, msg, ROUTING, timing_inf.timing_analysis_enabled);

	if(timing_inf.timing_analysis_enabled)
	{
		#ifdef CREATE_ECHO_FILES
			print_timing_graph_as_blif("post_flow_timing_graph.blif", models);
		#endif
	    free_timing_graph(net_slack);
	    free_net_delay(net_delay, &net_delay_chunk_list_head);
	}

    free_route_structs(clb_opins_used_locally);
    free_saved_routing(best_routing, saved_clb_opins_used_locally);
    fflush(stdout);

    return (final);
}


void
init_chan(int cfactor,
	  t_chan_width_dist chan_width_dist)
{

/* Assigns widths to channels (in tracks).  Minimum one track          * 
 * per channel.  io channels are io_rat * maximum in interior          * 
 * tracks wide.  The channel distributions read from the architecture  *
 * file are scaled by cfactor.                                         */

    float x, separation, chan_width_io;
    int nio, i;
    t_chan chan_x_dist, chan_y_dist;

    chan_width_io = chan_width_dist.chan_width_io;
    chan_x_dist = chan_width_dist.chan_x_dist;
    chan_y_dist = chan_width_dist.chan_y_dist;

/* io channel widths */

    nio = (int)floor(cfactor * chan_width_io + 0.5);
    if(nio == 0)
	nio = 1;		/* No zero width channels */

    chan_width_x[0] = chan_width_x[ny] = nio;
    chan_width_y[0] = chan_width_y[nx] = nio;

    if(ny > 1)
	{
	    separation = 1. / (ny - 2.);	/* Norm. distance between two channels. */
	    x = 0.;		/* This avoids div by zero if ny = 2. */
	    chan_width_x[1] = (int)floor(cfactor * comp_width(&chan_x_dist, x,
							      separation) +
					 0.5);

	    /* No zero width channels */
	    chan_width_x[1] = max(chan_width_x[1], 1);

	    for(i = 1; i < ny - 1; i++)
		{
		    x = (float)i / ((float)(ny - 2.));
		    chan_width_x[i + 1] =
			(int)floor(cfactor *
				   comp_width(&chan_x_dist, x,
					      separation) + 0.5);
		    chan_width_x[i + 1] = max(chan_width_x[i + 1], 1);
		}
	}

    if(nx > 1)
	{
	    separation = 1. / (nx - 2.);	/* Norm. distance between two channels. */
	    x = 0.;		/* Avoids div by zero if nx = 2. */
	    chan_width_y[1] = (int)floor(cfactor * comp_width(&chan_y_dist, x,
							      separation) +
					 0.5);

	    chan_width_y[1] = max(chan_width_y[1], 1);

	    for(i = 1; i < nx - 1; i++)
		{
		    x = (float)i / ((float)(nx - 2.));
		    chan_width_y[i + 1] =
			(int)floor(cfactor *
				   comp_width(&chan_y_dist, x,
					      separation) + 0.5);
		    chan_width_y[i + 1] = max(chan_width_y[i + 1], 1);
		}
	}
#ifdef VERBOSE
    printf("\nchan_width_x:\n");
    for(i = 0; i <= ny; i++)
	printf("%d  ", chan_width_x[i]);
    printf("\n\nchan_width_y:\n");
    for(i = 0; i <= nx; i++)
	printf("%d  ", chan_width_y[i]);
    printf("\n\n");
#endif

}


static float
comp_width(t_chan * chan,
	   float x,
	   float separation)
{

/* Return the relative channel density.  *chan points to a channel   *
 * functional description data structure, and x is the distance      *   
 * (between 0 and 1) we are across the chip.  separation is the      *   
 * distance between two channels, in the 0 to 1 coordinate system.   */

    float val;

    switch (chan->type)
	{

	case UNIFORM:
	    val = chan->peak;
	    break;

	case GAUSSIAN:
	    val = (x - chan->xpeak) * (x - chan->xpeak) / (2 * chan->width *
							   chan->width);
	    val = chan->peak * exp(-val);
	    val += chan->dc;
	    break;

	case PULSE:
	    val = (float)fabs((double)(x - chan->xpeak));
	    if(val > chan->width / 2.)
		{
		    val = 0;
		}
	    else
		{
		    val = chan->peak;
		}
	    val += chan->dc;
	    break;

	case DELTA:
	    val = x - chan->xpeak;
	    if(val > -separation / 2. && val <= separation / 2.)
		val = chan->peak;
	    else
		val = 0.;
	    val += chan->dc;
	    break;

	default:
	    printf("Error in comp_width:  Unknown channel type %d.\n",
		   chan->type);
	    exit(1);
	    break;
	}

    return (val);
}

/* After placement, logical pins for blocks, and nets must be updated to correspond with physical pins of type */
/* This function should only be called once */
void
post_place_sync(INP int num_blocks,
		INOUTP const struct s_block block_list[])
{
    int iblk, j, k, inet;
    t_type_ptr type;
    int max_num_block_pins;

    /* Go through each block */
    for(iblk = 0; iblk < num_blocks; ++iblk)
	{
	    type = block[iblk].type;
	    assert(type->num_pins % type->capacity == 0);
	    max_num_block_pins = type->num_pins / type->capacity;
	    /* Logical location and physical location is offset by z * max_num_block_pins */
	    /* Sync blocks and nets */
	    for(j = 0; j < max_num_block_pins; j++)
		{
		    inet = block[iblk].nets[j];
		    if(inet != OPEN && block[iblk].z > 0)
			{
			    assert(block[iblk].
				   nets[j +
					block[iblk].z * max_num_block_pins] ==
				   OPEN);
			    block[iblk].nets[j +
					     block[iblk].z *
					     max_num_block_pins] =
				block[iblk].nets[j];
			    block[iblk].nets[j] = OPEN;
				for(k = 0; k <= clb_net[inet].num_sinks; k++)
				{
				    if(clb_net[inet].node_block[k] == iblk)
					{
					    assert(clb_net[inet].
						   node_block_pin[k] == j);
					    clb_net[inet].node_block_pin[k] =
						j +
						block[iblk].z *
						max_num_block_pins;
					    break;
					}
				}
				assert(k <= clb_net[inet].num_sinks);
			}
		}
	}
}

void
free_pb_data(t_pb *pb)
{
	int i, j;
	const t_pb_type *pb_type;
	t_rr_node *temp;

	if(pb == NULL || pb->name == NULL) {
		return;
	}

	pb_type = pb->pb_graph_node->pb_type;

	/* free existing rr graph for pb */
	if(pb->rr_graph) {
		temp = rr_node;
		rr_node = pb->rr_graph;
		num_rr_nodes = pb->pb_graph_node->total_pb_pins;
		free_rr_graph();
		rr_node = temp;
	}

	if(pb_type->num_modes > 0) {
		/* Free children of pb */
		for(i = 0; i < pb_type->modes[pb->mode].num_pb_type_children; i++) {
			for(j = 0; j < pb_type->modes[pb->mode].pb_type_children[i].num_pb; j++) {
				if(pb->child_pbs[i]) {
					free_pb_data(&pb->child_pbs[i][j]);
				}
			}
		}
	}

	/* Frees all the pb data structures.                                 */
	if(pb->name) {
		free(pb->name);
		if(pb->child_pbs) {
			free(pb->child_pbs);
		}
	}
}
