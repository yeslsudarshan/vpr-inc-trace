#include <stdio.h>
#include "vpr_types.h"
#include "globals.h"
#include "inc_misc.h"
#include <string.h>
#include <assert.h>

/**
 * Write out the net name -> vnet mapping into <circuitname>.map
 */
void inc_dump_nets(struct s_file_name_opts *FileNameOpts)
{
	FILE *fp;
	int vnet, inet, iblk;

	inc_infer_vpack_blocks_and_pins();

	char *map_file = malloc(sizeof(char)*strlen(FileNameOpts->CircuitName)+strlen(".map")+1);
	strcpy(map_file, FileNameOpts->CircuitName);
	strcat(map_file, ".map");
	fp = fopen(map_file, "w");
	for (vnet = 0; vnet < num_logical_nets; vnet++)
	{
		inet = vpack_to_clb_net_mapping[vnet];
		if (inet == OPEN)
			iblk = vpack_net[vnet].node_block[0];
		else
			iblk = clb_net[inet].node_block[0];
		if ((inet == OPEN || !clb_net[inet].is_global) && iblk != OPEN)
			fprintf(fp, "%d,%s,%d,%d\n", vnet, vpack_net[vnet].name, block[iblk].x, block[iblk].y);
	}
	fclose(fp);
	free(map_file);
}

/**
 * Infer each vpack_net with node_block[] and node_block_pins[] 
 * from its block's pb structure
 */
void inc_infer_vpack_blocks_and_pins(void)
{
	int iblk;

	assert(strcmp(type_descriptors[2].name, "clb") == 0);
	for (iblk = 0; iblk < num_blocks; iblk++)
	{
		if (block[iblk].type->index == 2) /* clb */
		{
			int ible, nbles;
			assert(block[iblk].pb->pb_graph_node->pb_type->num_pb == 1);
			assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);
			assert(block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children == 1);
			nbles = block[iblk].pb->pb_graph_node->pb_type->modes[0].pb_type_children[0].num_pb;
			for (ible = 0; ible < nbles; ible++)
			{
				int ipin, npins;
				assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_ports == 1);
				npins = block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_pins[0];
				for (ipin = 0; ipin < npins; ipin++)
				{
					int ptc, inet;
					/*assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_pins[0] == 1);*/
					/* BLE OPIN ptc */
					ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
						.pin_count_in_cluster;
					inet = block[iblk].pb->rr_graph[ptc].net_num;
					if (inet == OPEN) continue;

					assert(block[iblk].pb->pb_graph_node->num_output_ports == 1);
					/*assert(block[iblk].pb->pb_graph_node->num_output_pins[0] == 10);*/
					/* CLB OPIN */
					ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*npins+ipin].pin_count_in_cluster;

					if (vpack_net[inet].node_block[0] != iblk)
					{
						assert(vpack_net[inet].node_block[0] == OPEN);
						vpack_net[inet].node_block[0] = iblk;
					}

					if (vpack_net[inet].node_block_pin[0] != ptc)
					{
						assert(vpack_net[inet].node_block_pin[0] == OPEN);
						vpack_net[inet].node_block_pin[0] = ptc;
					}
				}
			}
		}
	}

}
