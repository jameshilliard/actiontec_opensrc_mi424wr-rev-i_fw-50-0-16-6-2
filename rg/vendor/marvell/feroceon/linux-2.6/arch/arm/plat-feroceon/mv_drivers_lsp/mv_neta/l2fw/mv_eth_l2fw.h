/* l2fw/mv_eth_l2fw.h */

#ifndef L2FW_MV_ETH_L2FW_H
#define L2FW_MV_ETH_L2FW_H

#include "mvOs.h"
#include "mv_neta/net_dev/mv_netdev.h"

#define	L2FW_HASH_SIZE   (1 << 16)

struct eth_port_l2fw {
	int cmd;
	int txPort;
};

void l2fw(int cmd, int rx_port, int tx_port);
void l2fw_xor(int threshold);
int mv_eth_rx_l2f(struct eth_port *pp, int rx_todo, int rxq);

#endif
