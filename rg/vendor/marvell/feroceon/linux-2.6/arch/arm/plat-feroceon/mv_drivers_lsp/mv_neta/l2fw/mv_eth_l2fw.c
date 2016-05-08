/* mv_eth_l2fw.c */
#include <linux/ctype.h>

#include "xor/mvXor.h"
#include "xor/mvXorRegs.h"
#include "mv_hal_if/mvSysXorApi.h"

#include "mvOs.h"
#include "mv_eth_l2fw.h"
#include "mv_neta/net_dev/mv_netdev.h"
#include "gbe/mvNeta.h"
#include "gbe/mvNetaRegs.h"
#include "nfp/mvNfp.h"
#include "mv_eth_l2fw.h"
#include "ctrlEnv/mvCtrlEnvLib.h"

static int mv_eth_ports_l2fw_num;

struct eth_port_l2fw **mv_eth_ports_l2fw;

static inline
struct neta_tx_desc *mv_eth_tx_desc_get(struct tx_queue *txq_ctrl, int num)
{
	/* Is enough TX descriptors to send packet */
	if ((txq_ctrl->txq_count + num) >= txq_ctrl->txq_size) {
		/*
		printk("eth_tx: txq_ctrl->txq=%d - no_resource: txq_count=%d,
				txq_size=%d, num=%d\n",
				txq_ctrl->txq, txq_ctrl->txq_count,
				txq_ctrl->txq_size, num);
		*/
		STAT_DBG(txq_ctrl->stats.txq_err++);
		return NULL;
	}
	return mvNetaTxqNextDescGet(txq_ctrl->q);
}


static int mv_ctrl_txdone = CONFIG_MV_ETH_TXDONE_COAL_PKTS;
static void dump_xor(void)
{
	mvOsPrintf(" CHANNEL_ARBITER_REG %08x\n",
		MV_REG_READ(XOR_CHANNEL_ARBITER_REG(1)));
	mvOsPrintf(" CONFIG_REG          %08x\n",
		MV_REG_READ(XOR_CONFIG_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" ACTIVATION_REG      %08x\n",
		MV_REG_READ(XOR_ACTIVATION_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" CAUSE_REG           %08x\n",
		MV_REG_READ(XOR_CAUSE_REG(1)));
	mvOsPrintf(" MASK_REG            %08x\n",
		MV_REG_READ(XOR_MASK_REG(1)));
	mvOsPrintf(" ERROR_CAUSE_REG     %08x\n",
		MV_REG_READ(XOR_ERROR_CAUSE_REG(1)));
	mvOsPrintf(" ERROR_ADDR_REG      %08x\n",
		MV_REG_READ(XOR_ERROR_ADDR_REG(1)));
	mvOsPrintf(" NEXT_DESC_PTR_REG   %08x\n",
		MV_REG_READ(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" CURR_DESC_PTR_REG   %08x\n",
		MV_REG_READ(XOR_CURR_DESC_PTR_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" BYTE_COUNT_REG      %08x\n\n",
		MV_REG_READ(XOR_BYTE_COUNT_REG(1, XOR_CHAN(0))));
}


/* L2fw defines */
#define L2FW_DISABLE				0
#define TX_AS_IS					1
#define SWAP_MAC					2
#define COPY_AND_SWAP		        3

#define XOR_CAUSE_DONE_MASK(chan) ((BIT0|BIT1) << (chan * 16))

static int         l2fw_xor_threshold = 200;
static MV_XOR_DESC *eth_xor_desc = NULL;
static MV_LONG      eth_xor_desc_phys_addr;


static int mv_eth_poll_l2fw(struct napi_struct *napi, int budget)
{
	int rx_done = 0;
	MV_U32 causeRxTx;

	struct eth_port *pp =
		container_of(napi, struct eth_port, napi[smp_processor_id()]);
#ifdef CONFIG_MV_ETH_DEBUG_CODE
	if (pp->flags & MV_ETH_F_DBG_POLL) {
		mvOsPrintf("%s_%d ENTER: port=%d, mask=0x%x,cause=0x%x\n",
				__func__, pp->stats.poll, pp->port,
				MV_REG_READ(NETA_INTR_NEW_MASK_REG(pp->port)),
				MV_REG_READ(NETA_INTR_NEW_CAUSE_REG(pp->port)));
	}
#endif /* CONFIG_MV_ETH_DEBUG_CODE */
	int txPort = mv_eth_ports_l2fw[pp->port]->txPort;
	read_lock(&pp->rwlock);
	read_lock(&mv_eth_ports[txPort]->rwlock);

	STAT_INFO(pp->stats.poll++);

	/* Read cause register */
	causeRxTx = MV_REG_READ(NETA_INTR_NEW_CAUSE_REG(pp->port)) &
	    (MV_ETH_MISC_SUM_INTR_MASK | MV_ETH_TXDONE_INTR_MASK |
		 MV_ETH_RX_INTR_MASK);

	if (causeRxTx & MV_ETH_MISC_SUM_INTR_MASK) {
		MV_U32 causeMisc;

		/* Process MISC events - Link, etc ??? */
		causeRxTx &= ~MV_ETH_MISC_SUM_INTR_MASK;
		causeMisc = MV_REG_READ(NETA_INTR_MISC_CAUSE_REG(pp->port));

		if (causeMisc & NETA_CAUSE_LINK_CHANGE_MASK)
			mv_eth_link_event(pp, 1);
		MV_REG_WRITE(NETA_INTR_MISC_CAUSE_REG(pp->port), 0);
	}

	causeRxTx |= pp->causeRxTx[smp_processor_id()];
#ifdef CONFIG_MV_ETH_TXDONE_ISR
	if (causeRxTx & MV_ETH_TXDONE_INTR_MASK) {
		int tx_todo = 0;

		/* TX_DONE process */
		if (MV_PON_PORT(pp->port))
			mv_eth_tx_done_pon(pp, &tx_todo);
		else
			mv_eth_tx_done_gbe(pp, (causeRxTx & MV_ETH_TXDONE_INTR_MASK), &tx_todo);

		causeRxTx &= ~MV_ETH_TXDONE_INTR_MASK;
	}
#endif /* CONFIG_MV_ETH_TXDONE_ISR */

#if (CONFIG_MV_ETH_RXQ > 1)
	while ((causeRxTx != 0) && (budget > 0)) {
		int count, rx_queue;

		rx_queue = mv_eth_rx_policy(causeRxTx);
		if (rx_queue == -1)
			break;

		count = mv_eth_rx_l2f(pp, budget, rx_queue);
		rx_done += count;
		budget -= count;
		if (budget > 0)
			causeRxTx &=
			 ~((1 << rx_queue) << NETA_CAUSE_RXQ_OCCUP_DESC_OFFS);
	}
#else
	rx_done = mv_eth_rx_l2f(pp, budget, CONFIG_MV_ETH_RXQ_DEF);
	budget -= rx_done;
#endif /* (CONFIG_MV_ETH_RXQ > 1) */


	if (budget > 0) {
		unsigned long flags;

		causeRxTx = 0;

		napi_complete(&pp->napi[smp_processor_id()]);
		STAT_INFO(pp->stats.poll_exit++);

		local_irq_save(flags);
		MV_REG_WRITE(NETA_INTR_NEW_MASK_REG(pp->port),
			(MV_ETH_MISC_SUM_INTR_MASK | MV_ETH_TXDONE_INTR_MASK |
				  MV_ETH_RX_INTR_MASK));

		local_irq_restore(flags);
	}
	pp->causeRxTx[smp_processor_id()] = causeRxTx;

	read_lock(&mv_eth_ports[txPort]->rwlock);
	read_unlock(&pp->rwlock);

	return rx_done;
}


void mv_eth_set_l2fw(int enable, int cmd, int rx_port, int out_tx_port)
{
	int cpu;
	struct eth_port *pp;
	struct net_device *dev;

	mvOsPrintf("Setting L2FW: cmd=%d in %s\n",  cmd , __func__);
	pp     = mv_eth_ports[rx_port];
	if (!pp) {
		mvOsPrintf("pp is NULL in setting L2FW (%s)\n", __func__);
		return;
	}

	dev = pp->dev;
	if (dev == NULL) {
		mvOsPrintf("device is NULL in in setting L2FW (%s)\n", __func__);
		return;
	}
	if (!test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags))) {
		mvOsPrintf("Device is down for port=%d ; MV_ETH_F_STARTED_BIT is not set in %s\n", rx_port, __func__);
		mvOsPrintf("Cannot set to L2FW mode in %s\n", __func__);
		return;
	}
	for_each_possible_cpu(cpu)
		{
		if (cmd == L2FW_DISABLE) {
			mvOsPrintf("L2FW_DISABLE) cmd = %d in %s\n",
					cmd, __func__);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_disable(&pp->napi[cpu]);
			netif_napi_del(&pp->napi[cpu]);
			netif_napi_add(dev, &pp->napi[cpu], mv_eth_poll,
				CONFIG_MV_ETH_RXQ_DESC / 2);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_enable(&pp->napi[cpu]);
		} else {
			mvOsPrintf("Enable l2fw:cmd=%d port=%d in %s\n",
				 cmd, pp->port, __func__);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_disable(&pp->napi[cpu]);
			netif_napi_del(&pp->napi[cpu]);
			netif_napi_add(dev, &pp->napi[cpu], mv_eth_poll_l2fw,
				CONFIG_MV_ETH_RXQ_DESC / 2);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_enable(&pp->napi[cpu]);
			}
		}
}


static inline struct eth_pbuf *l2fw_swap_mac(struct eth_pbuf *pRxPktInfo)
{
	MV_U16 *pSrc;
	int i;
	MV_U16 swap;
	pSrc = (MV_U16 *)(pRxPktInfo->pBuf + pRxPktInfo->offset + MV_ETH_MH_SIZE);

	for (i = 0; i < 3; i++) {
		swap = pSrc[i];
		pSrc[i] = pSrc[i+3];
		pSrc[i+3] = swap;
		}

	return  pRxPktInfo;
}

static inline void l2fw_copy_mac(struct eth_pbuf *pRxPktInfo,
					 struct eth_pbuf *pTxPktInfo)
	{
	/* copy 30 bytes (start after MH header) */
    /* 12 for SA + DA */
	/* 18 for the rest */
	MV_U16 *pSrc;
	MV_U16 *pDst;
	int i;
	pSrc = (MV_U16 *)(pRxPktInfo->pBuf + pRxPktInfo->offset + MV_ETH_MH_SIZE);
	pDst = (MV_U16 *)(pTxPktInfo->pBuf + pTxPktInfo->offset + MV_ETH_MH_SIZE);
	/* swap mac SA and DA */
	for (i = 0; i < 3; i++) {
		pDst[i]   = pSrc[i+3];
		pDst[i+3] = pSrc[i];
		}
	for (i = 6; i < 15; i++)
		pDst[i] = pSrc[i];
	}

static inline void l2fw_copy_and_swap_mac(struct eth_pbuf *pRxPktInfo, struct eth_pbuf *pTxPktInfo)
{
	MV_U16 *pSrc;
	MV_U16 *pDst;
	int i;

	pSrc = (MV_U16 *)(pRxPktInfo->pBuf +  pRxPktInfo->offset + MV_ETH_MH_SIZE);
	pDst = (MV_U16 *)(pTxPktInfo->pBuf +  pTxPktInfo->offset + MV_ETH_MH_SIZE);
	for (i = 0; i < 3; i++) {
		pDst[i]   = pSrc[i+3];
		pDst[i+3] = pSrc[i];
		}
}

static inline
struct eth_pbuf *eth_l2fw_copy_packet_withoutXor(struct eth_pbuf *pRxPktInfo)
{
	MV_U8 *pSrc;
	MV_U8 *pDst;
	struct bm_pool *pool;
	struct eth_pbuf *pTxPktInfo;

	mvOsCacheInvalidate(NULL, pRxPktInfo->pBuf + pRxPktInfo->offset,
						pRxPktInfo->bytes);

	pool = &mv_eth_pool[pRxPktInfo->pool];
	pTxPktInfo = mv_eth_pool_get(pool);
	if (pTxPktInfo == NULL) {
		mvOsPrintf("pTxPktInfo == NULL in %s\n", __func__);
		return NULL;
		}
	pSrc = pRxPktInfo->pBuf +  pRxPktInfo->offset + MV_ETH_MH_SIZE;
	pDst = pTxPktInfo->pBuf +  pTxPktInfo->offset + MV_ETH_MH_SIZE;

	memcpy(pDst+12, pSrc+12, pRxPktInfo->bytes-12);
	l2fw_copy_and_swap_mac(pRxPktInfo, pTxPktInfo);
	pTxPktInfo->bytes = pRxPktInfo->bytes;
	mvOsCacheFlush(NULL, pTxPktInfo->pBuf + pTxPktInfo->offset, pTxPktInfo->bytes);

	return pTxPktInfo;
}

static inline
struct eth_pbuf *eth_l2fw_copy_packet_withXor(struct eth_pbuf *pRxPktInfo)
{
	struct bm_pool *pool;
	struct eth_pbuf *pTxPktInfo;

	pool = &mv_eth_pool[pRxPktInfo->pool];
	pTxPktInfo = mv_eth_pool_get(pool);
	if (pTxPktInfo == NULL) {
		mvOsPrintf("pTxPktInfo == NULL in %s\n", __func__);
		return NULL;
		}
	eth_xor_desc->srcAdd0    = pRxPktInfo->physAddr + pRxPktInfo->offset + MV_ETH_MH_SIZE + 30;
	eth_xor_desc->phyDestAdd = pTxPktInfo->physAddr + pTxPktInfo->offset + MV_ETH_MH_SIZE + 30;

	eth_xor_desc->byteCnt    = pRxPktInfo->bytes - 30;

	eth_xor_desc->phyNextDescPtr = 0;
	eth_xor_desc->status         = BIT31;
	/* we had changed only the first part of eth_xor_desc, so flush only one
	 line of cache */
	mvOsCacheLineFlush(NULL, eth_xor_desc);

	MV_REG_WRITE(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0)), eth_xor_desc_phys_addr);

	MV_REG_WRITE(XOR_ACTIVATION_REG(1, XOR_CHAN(0)), XEXACTR_XESTART_MASK);

	mvOsCacheLineInv(NULL, pRxPktInfo->pBuf + pRxPktInfo->offset);
	l2fw_copy_mac(pRxPktInfo, pTxPktInfo);
	mvOsCacheLineFlush(NULL, pTxPktInfo->pBuf + pTxPktInfo->offset);

    /* Update TxPktInfo */
	pTxPktInfo->bytes = pRxPktInfo->bytes;
	return pTxPktInfo;
}

void setXorDesc(void)
{
	unsigned int mode;
	eth_xor_desc = mvOsMalloc(sizeof(MV_XOR_DESC) + XEXDPR_DST_PTR_DMA_MASK + 32);
	eth_xor_desc = (MV_XOR_DESC *)MV_ALIGN_UP((MV_U32)eth_xor_desc, XEXDPR_DST_PTR_DMA_MASK+1);
	eth_xor_desc_phys_addr = mvOsIoVirtToPhys(NULL, eth_xor_desc);
	mvSysXorInit();

	mode = MV_REG_READ(XOR_CONFIG_REG(1, XOR_CHAN(0)));
	mode &= ~XEXCR_OPERATION_MODE_MASK;
	mode |= XEXCR_OPERATION_MODE_DMA;
	MV_REG_WRITE(XOR_CONFIG_REG(1, XOR_CHAN(0)), mode);

    MV_REG_WRITE(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0)), eth_xor_desc_phys_addr);
	dump_xor();
}




static inline int xorReady(void)
{
	int timeout = 0;

	while (!(MV_REG_READ(XOR_CAUSE_REG(1)) & XOR_CAUSE_DONE_MASK(XOR_CHAN(0)))) {
		if (timeout > 0x100000) {
			mvOsPrintf("XOR timeout\n");
			return 0;
			}
		timeout++;
	}

	/* Clear int */
	MV_REG_WRITE(XOR_CAUSE_REG(1), ~(XOR_CAUSE_DONE_MASK(XOR_CHAN(0))));

	return 1;
}



void l2fw(int cmd, int rx_port, int tx_port)
{
	struct eth_port_l2fw *ppl2fw;
	ppl2fw = mv_eth_ports_l2fw[rx_port];
	mvOsPrintf("in %s cmd=%d rx_port=%d tx_port=%d\n", __func__,
				cmd, rx_port, tx_port);
	if (ppl2fw == NULL) {
		mvOsPrintf("ppl2fw is NULL in %s\n", __func__);
		return;
	} else
		mvOsPrintf("ppl2fw is NOT NULL in %s\n", __func__);


	/* for not getting linux traffic such as ARP */

	ppl2fw->txPort = tx_port;
	ppl2fw->cmd	= cmd;
	if (cmd == L2FW_DISABLE) {
		mvOsPrintf("disabling l2fw in %s\n", __func__);
		mv_eth_set_l2fw(0, cmd, rx_port, tx_port);
	} else {
		mvOsPrintf("Enabling l2fw in %s\n", __func__);
		mv_eth_set_l2fw(1, cmd, rx_port, tx_port);
	}

}



void l2fw_xor(int threshold)
{
	mvOsPrintf("setting threshold to %d in %s\n", threshold, __func__);
	l2fw_xor_threshold = threshold;
}


static inline MV_STATUS mv_eth_l2fw_tx(struct eth_pbuf *pkt, struct eth_port *pp, int withXor)
{
	struct neta_tx_desc *tx_desc;
	u32 tx_cmd = 0;
	struct tx_queue *txq_ctrl;
/* TBD: we can skip taking the rwlock for performance , assuming that
 nobody from control plane stopps the nic or changes the MTU on the fly */
	int txq = 0; /* TBD: use mv_eth_txq_tos_map_get(pp, pkt->tos); */


	txq_ctrl = &pp->txq_ctrl[pp->txp * CONFIG_MV_ETH_TXQ + txq];
	spin_lock(&txq_ctrl->queue_lock);

#ifndef CONFIG_MV_ETH_TXDONE_ISR
	if (txq_ctrl->txq_count >= mv_ctrl_txdone)
		mv_eth_txq_done(pp, txq_ctrl);
#endif

	/* Get next descriptor for tx, single buffer, so FIRST & LAST */
	tx_desc = mv_eth_tx_desc_get(txq_ctrl, 1);
	if (tx_desc == NULL) {
		/* No resources: Drop */
		spin_unlock(&txq_ctrl->queue_lock);
		pp->dev->stats.tx_dropped++;
		if (withXor)
			xorReady();

		return MV_DROPPED;
	}
	txq_ctrl->txq_count++;

	tx_cmd |= NETA_TX_BM_ENABLE_MASK | NETA_TX_BM_POOL_ID_MASK(pkt->pool);
	txq_ctrl->shadow_txq[txq_ctrl->shadow_txq_put_i] = (u32) NULL;
	mv_eth_shadow_inc_put(txq_ctrl);

	tx_desc->command = tx_cmd | NETA_TX_L4_CSUM_NOT |
		NETA_TX_FLZ_DESC_MASK | NETA_TX_F_DESC_MASK
		| NETA_TX_L_DESC_MASK |
		NETA_TX_PKT_OFFSET_MASK(pkt->offset + MV_ETH_MH_SIZE);

	tx_desc->dataSize    = pkt->bytes;
	tx_desc->bufPhysAddr = pkt->physAddr;

#ifdef CONFIG_MV_ETH_DEBUG_CODE
	if (pp->flags & MV_ETH_F_DBG_TX) {
		mvOsPrintf("\n");
		mv_eth_tx_desc_print(tx_desc);
		mv_eth_pkt_print(pkt);
	}
#endif /* CONFIG_MV_ETH_DEBUG_CODE */

	mv_eth_tx_desc_flush(tx_desc);

	if (withXor) {
		if (!xorReady()) {
			mvOsPrintf("MV_DROPPED in %s\n", __func__);
			spin_unlock(&txq_ctrl->queue_lock);
			return MV_DROPPED;
		}
	}
	mvNetaTxqPendDescAdd(pp->port, pp->txp, 0, 1);
	spin_unlock(&txq_ctrl->queue_lock);

	return MV_OK;
}

int mv_eth_rx_l2f(struct eth_port *pp, int rx_todo, int rxq)
{
	struct eth_port  *new_pp;
	MV_NETA_RXQ_CTRL *rx_ctrl = pp->rxq_ctrl[rxq].q;
	int rx_done, rx_filled;
	struct neta_rx_desc *rx_desc;
	u32 rx_status = MV_OK;
	struct eth_pbuf *pkt;
	struct eth_pbuf *newpkt = NULL;
	struct bm_pool *pool;
	MV_STATUS status = MV_OK;
	struct eth_port_l2fw *ppl2fw = mv_eth_ports_l2fw[pp->port];

	/* Get number of received packets */
	rx_done = mvNetaRxqBusyDescNumGet(pp->port, rxq);
	if (rx_todo > rx_done)
		rx_todo = rx_done;

	rx_done = 0;
	rx_filled = 0;

	/* Fairness NAPI loop */
	while (rx_done < rx_todo) {

#ifdef CONFIG_MV_ETH_RX_DESC_PREFETCH
		rx_desc = mv_eth_rx_prefetch(pp, rx_ctrl, rx_done, rx_todo);
#else
		rx_desc = mvNetaRxqNextDescGet(rx_ctrl);
		mvOsCacheLineInv(NULL, rx_desc);
		prefetch(rx_desc);
#endif /* CONFIG_MV_ETH_RX_DESC_PREFETCH */

		rx_done++;
		rx_filled++;

		pkt = (struct eth_pbuf *)rx_desc->bufCookie;

		pool = &mv_eth_pool[pkt->pool];

		rx_status = rx_desc->status;
		if (((rx_status & NETA_RX_FL_DESC_MASK) != NETA_RX_FL_DESC_MASK) ||
			(rx_status & NETA_RX_ES_MASK)) {

			STAT_ERR(pp->stats.rx_error++);

			if (pp->dev)
				pp->dev->stats.rx_errors++;

			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
			continue;
		}

		pkt->bytes = rx_desc->dataSize - (MV_ETH_CRC_SIZE + MV_ETH_MH_SIZE);

		new_pp  = mv_eth_ports[ppl2fw->txPort];
		switch (ppl2fw->cmd) {
		case TX_AS_IS:
				status = mv_eth_l2fw_tx(pkt, new_pp, 0);
				break;

		case SWAP_MAC:
				mvOsCacheLineInv(NULL, pkt->pBuf + pkt->offset);
				l2fw_swap_mac(pkt);
				mvOsCacheLineFlush(NULL, pkt->pBuf+pkt->offset);
				status = mv_eth_l2fw_tx(pkt, new_pp, 0);
				break;

		case COPY_AND_SWAP:
				if (pkt->bytes >= l2fw_xor_threshold) {
					newpkt = eth_l2fw_copy_packet_withXor(pkt);
					if (newpkt)
						status = mv_eth_l2fw_tx(newpkt, new_pp, 1);
					else
						status = MV_ERROR;
				} else {
						newpkt = eth_l2fw_copy_packet_withoutXor(pkt);
						if (newpkt)
							status = mv_eth_l2fw_tx(newpkt, new_pp, 0);
						else
							status = MV_ERROR;
				}
		}

		if (status == MV_OK) {
			mvOsCacheLineInv(NULL, rx_desc);

			if (ppl2fw->cmd	== COPY_AND_SWAP)
				mv_eth_pool_put(pool, pkt);
			continue;
		} else if (status == MV_DROPPED) {
			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
			if (ppl2fw->cmd	== COPY_AND_SWAP)
				mv_eth_pool_put(pool, newpkt);

			continue;
		} else if (status == MV_ERROR)
			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);

	} /* of while */

	mvNetaRxqDescNumUpdate(pp->port, rxq, rx_done, rx_filled);

	return rx_done;
}

#ifdef CONFIG_MV_ETH_L2FW
int __devinit mv_l2fw_init(void)
{
	int size;
	int port;

	mv_eth_ports_l2fw_num = mvCtrlEthMaxPortGet();
	mvOsPrintf("mv_eth_ports_l2fw_num=%d in %s\n", mv_eth_ports_l2fw_num, __func__);
	size = mv_eth_ports_l2fw_num * sizeof(struct eth_port_l2fw *);
	mv_eth_ports_l2fw = mvOsMalloc(size);
	if (!mv_eth_ports_l2fw)
		goto oom;
	memset(mv_eth_ports_l2fw, 0, size);
	for (port = 0; port < mv_eth_ports_l2fw_num; port++) {
		mv_eth_ports_l2fw[port] =
			mvOsMalloc(sizeof(struct eth_port_l2fw));
		if (!mv_eth_ports_l2fw[port])
			goto oom1;
		mv_eth_ports_l2fw[port]->cmd    = L2FW_DISABLE;
		mv_eth_ports_l2fw[port]->txPort = -1;
	}
	return 0;
oom:
	mvOsPrintf("%s: out of memory in L2FW initialization\n", __func__);
oom1:
	mvOsFree(mv_eth_ports_l2fw);
	return -ENOMEM;

}
#endif

module_init(mv_l2fw_init);

MODULE_AUTHOR("Rami Rosen");
MODULE_DESCRIPTION("l2fw module");
MODULE_LICENSE("GPL");

