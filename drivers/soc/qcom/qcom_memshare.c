// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal mainline port of the Qualcomm memshare QMI service.
 *
 * Based on msm_memshare.c / heap_mem_ext_v01.c from the msm-4.19 vendor
 * kernel, Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 *
 * Registers QMI service 0x34 (MEM_SHARE) on APPS. Remote processors (the
 * modem) ask it for physically contiguous DDR; the buffer is allocated
 * here and shared with the modem VM through SCM.
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>

#define MEM_SHARE_SERVICE_SVC_ID	0x34
#define MEM_SHARE_SERVICE_VERS		1
#define MEM_SHARE_SERVICE_INS_ID	1

#define MEM_ALLOC_GENERIC_REQ_MSG_V01	0x0022
#define MEM_ALLOC_GENERIC_RESP_MSG_V01	0x0022
#define MEM_FREE_GENERIC_REQ_MSG_V01	0x0023
#define MEM_FREE_GENERIC_RESP_MSG_V01	0x0023
#define MEM_QUERY_SIZE_REQ_MSG_V01	0x0024
#define MEM_QUERY_SIZE_RESP_MSG_V01	0x0024

#define MEM_FREE_REQ_MAX_MSG_LEN_V01	255
#define MEM_QUERY_MAX_MSG_LEN_V01	255
#define MAX_ARR_CNT_V01			64

#define DHMS_MEM_PROC_MPSS_V01		0

#ifndef QMI_ERR_NO_MEMORY_V01
#define QMI_ERR_NO_MEMORY_V01		2
#endif
#ifndef QMI_ERR_INVALID_ID_V01
#define QMI_ERR_INVALID_ID_V01		41
#endif

/* ---- QMI message definitions (heap_mem_ext_v01) ---- */

struct dhms_mem_alloc_addr_info_type_v01 {
	u64 phy_addr;
	u32 num_bytes;
};

struct mem_alloc_generic_req_msg_v01 {
	u32 num_bytes;
	u32 client_id;
	u32 proc_id;
	u32 sequence_id;
	u8 alloc_contiguous_valid;
	u8 alloc_contiguous;
	u8 block_alignment_valid;
	u32 block_alignment;
};

struct mem_alloc_generic_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
	u8 sequence_id_valid;
	u32 sequence_id;
	u8 dhms_mem_alloc_addr_info_valid;
	u32 dhms_mem_alloc_addr_info_len;
	struct dhms_mem_alloc_addr_info_type_v01
		dhms_mem_alloc_addr_info[MAX_ARR_CNT_V01];
};

struct mem_free_generic_req_msg_v01 {
	u32 dhms_mem_alloc_addr_info_len;
	struct dhms_mem_alloc_addr_info_type_v01
		dhms_mem_alloc_addr_info[MAX_ARR_CNT_V01];
	u8 client_id_valid;
	u32 client_id;
	u8 proc_id_valid;
	u32 proc_id;
};

struct mem_free_generic_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};

struct mem_query_size_req_msg_v01 {
	u32 client_id;
	u8 proc_id_valid;
	u32 proc_id;
};

struct mem_query_size_rsp_msg_v01 {
	struct qmi_response_type_v01 resp;
	u8 size_valid;
	u32 size;
};

static const struct qmi_elem_info dhms_mem_alloc_addr_info_type_v01_ei[] = {
	{
		.data_type	= QMI_UNSIGNED_8_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u64),
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct dhms_mem_alloc_addr_info_type_v01, phy_addr),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct dhms_mem_alloc_addr_info_type_v01, num_bytes),
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_alloc_generic_req_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x01,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, num_bytes),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x02,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, client_id),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x03,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, proc_id),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x04,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, sequence_id),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, alloc_contiguous_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_1_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, alloc_contiguous),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, block_alignment_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_alloc_generic_req_msg_v01, block_alignment),
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_alloc_generic_resp_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct qmi_response_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x02,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01, resp),
		.ei_array	= qmi_response_type_v01_ei,
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01, sequence_id_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01, sequence_id),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01,
					   dhms_mem_alloc_addr_info_valid),
	},
	{
		.data_type	= QMI_DATA_LEN,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01,
					   dhms_mem_alloc_addr_info_len),
	},
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= MAX_ARR_CNT_V01,
		.elem_size	= sizeof(struct dhms_mem_alloc_addr_info_type_v01),
		.array_type	= VAR_LEN_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_alloc_generic_resp_msg_v01,
					   dhms_mem_alloc_addr_info),
		.ei_array	= dhms_mem_alloc_addr_info_type_v01_ei,
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_free_generic_req_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_DATA_LEN,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x01,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01,
					   dhms_mem_alloc_addr_info_len),
	},
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= MAX_ARR_CNT_V01,
		.elem_size	= sizeof(struct dhms_mem_alloc_addr_info_type_v01),
		.array_type	= VAR_LEN_ARRAY,
		.tlv_type	= 0x01,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01,
					   dhms_mem_alloc_addr_info),
		.ei_array	= dhms_mem_alloc_addr_info_type_v01_ei,
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01, client_id_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01, client_id),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01, proc_id_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x11,
		.offset		= offsetof(struct mem_free_generic_req_msg_v01, proc_id),
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_free_generic_resp_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct qmi_response_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x02,
		.offset		= offsetof(struct mem_free_generic_resp_msg_v01, resp),
		.ei_array	= qmi_response_type_v01_ei,
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_query_size_req_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x01,
		.offset		= offsetof(struct mem_query_size_req_msg_v01, client_id),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_query_size_req_msg_v01, proc_id_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_query_size_req_msg_v01, proc_id),
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info mem_query_size_resp_msg_data_v01_ei[] = {
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct qmi_response_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x02,
		.offset		= offsetof(struct mem_query_size_rsp_msg_v01, resp),
		.ei_array	= qmi_response_type_v01_ei,
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_query_size_rsp_msg_v01, size_valid),
	},
	{
		.data_type	= QMI_UNSIGNED_4_BYTE,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= 0x10,
		.offset		= offsetof(struct mem_query_size_rsp_msg_v01, size),
	},
	{
		.data_type	= QMI_EOTI,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
	},
};

/* ---- service state ---- */

#define MS_MAX_CLIENTS	8

struct ms_client {
	bool used;
	u32 client_id;
	u32 proc_id;
	void *va;
	dma_addr_t dma;
	size_t size;		/* allocated, page aligned */
	u32 req_size;		/* size the client asked for */
	u64 vmsrc;		/* current SCM owner bitmap */
	bool shared;		/* assigned to the modem */
	struct debugfs_blob_wrapper blob;	/* read-only view of the buffer */
	struct dentry *dbg;
};

/*
 * Size reported to MEM_QUERY_SIZE, indexed by client id.
 * Defaults follow lagoon.dtsi: client-id 1 = 5 MiB on request.
 */
static unsigned int query_size[4] = { 0, 0x500000, 0, 0 };
static int nr_query_size = ARRAY_SIZE(query_size);
module_param_array(query_size, uint, &nr_query_size, 0444);
MODULE_PARM_DESC(query_size, "Sizes reported for client ids 0..3");

static struct ms_client clients[MS_MAX_CLIENTS];
static DEFINE_MUTEX(ms_lock);
static struct dentry *ms_dbg_dir;

/*
 * Expose an allocated buffer as /sys/kernel/debug/memshare/client<id>-proc<p>.
 * The buffer is shared HLOS+MSS read/write, so APPS may read it at any time,
 * including after a modem crash (TrustZone does not scrub HLOS memory).
 */
static void ms_debugfs_add(struct ms_client *c)
{
	char name[32];

	if (IS_ERR_OR_NULL(ms_dbg_dir) || c->dbg)
		return;
	c->blob.data = c->va;
	c->blob.size = c->size;
	snprintf(name, sizeof(name), "client%u-proc%u", c->client_id, c->proc_id);
	c->dbg = debugfs_create_blob(name, 0400, ms_dbg_dir, &c->blob);
}

static void ms_debugfs_del(struct ms_client *c)
{
	debugfs_remove(c->dbg);
	c->dbg = NULL;
}
static struct platform_device *ms_pdev;
static struct qmi_handle ms_qmi;

static struct ms_client *ms_find(u32 client_id, u32 proc_id, bool create)
{
	int i;

	for (i = 0; i < MS_MAX_CLIENTS; i++)
		if (clients[i].used && clients[i].client_id == client_id &&
		    clients[i].proc_id == proc_id)
			return &clients[i];

	if (!create)
		return NULL;

	for (i = 0; i < MS_MAX_CLIENTS; i++) {
		if (!clients[i].used) {
			memset(&clients[i], 0, sizeof(clients[i]));
			clients[i].used = true;
			clients[i].client_id = client_id;
			clients[i].proc_id = proc_id;
			return &clients[i];
		}
	}
	return NULL;
}

static int ms_share_with_modem(struct ms_client *c)
{
	struct qcom_scm_vmperm perm[2] = {
		{ .vmid = QCOM_SCM_VMID_HLOS,    .perm = QCOM_SCM_PERM_RW },
		{ .vmid = QCOM_SCM_VMID_MSS_MSA, .perm = QCOM_SCM_PERM_RW },
	};
	int ret;

	c->vmsrc = BIT(QCOM_SCM_VMID_HLOS);
	ret = qcom_scm_assign_mem(c->dma, c->size, &c->vmsrc, perm, 2);
	if (ret) {
		pr_err("memshare: assign to modem failed: %d\n", ret);
		return ret;
	}
	c->shared = true;
	return 0;
}

static void ms_reclaim(struct ms_client *c)
{
	struct qcom_scm_vmperm perm = {
		.vmid = QCOM_SCM_VMID_HLOS, .perm = QCOM_SCM_PERM_RWX,
	};
	int ret;

	if (!c->shared)
		return;
	ret = qcom_scm_assign_mem(c->dma, c->size, &c->vmsrc, &perm, 1);
	if (ret)
		pr_err("memshare: reclaim failed: %d (leaking buffer)\n", ret);
	else
		c->shared = false;
}

static void ms_release(struct ms_client *c)
{
	if (!c->va)
		return;
	ms_debugfs_del(c);
	ms_reclaim(c);
	if (c->shared)
		return;		/* still owned by the modem, can't free */
	dma_free_coherent(&ms_pdev->dev, c->size, c->va, c->dma);
	c->va = NULL;
	c->size = 0;
}

static void ms_handle_alloc(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
			    struct qmi_txn *txn, const void *decoded)
{
	const struct mem_alloc_generic_req_msg_v01 *req = decoded;
	struct mem_alloc_generic_resp_msg_v01 *resp;
	struct ms_client *c;
	bool ok = false;
	int ret;

	pr_info("memshare: ALLOC client=%u proc=%u bytes=%u seq=%u contig=%d/%u align=%d/%u (from %u:%u)\n",
		req->client_id, req->proc_id, req->num_bytes, req->sequence_id,
		req->alloc_contiguous_valid, req->alloc_contiguous,
		req->block_alignment_valid, req->block_alignment,
		sq->sq_node, sq->sq_port);

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp)
		return;

	mutex_lock(&ms_lock);
	c = ms_find(req->client_id, req->proc_id, true);
	if (!c) {
		pr_err("memshare: client table full\n");
		goto respond;
	}

	if (!c->va) {
		size_t size = PAGE_ALIGN(req->num_bytes);

		if (!size)
			goto respond;
		c->va = dma_alloc_coherent(&ms_pdev->dev, size, &c->dma,
					   GFP_KERNEL);
		if (!c->va) {
			pr_err("memshare: allocation of %zu bytes failed (CMA too small?)\n",
			       size);
			goto respond;
		}
		c->size = size;
		if (req->proc_id == DHMS_MEM_PROC_MPSS_V01 &&
		    ms_share_with_modem(c)) {
			dma_free_coherent(&ms_pdev->dev, c->size, c->va, c->dma);
			c->va = NULL;
			c->size = 0;
			goto respond;
		}
	} else if (c->size < req->num_bytes) {
		pr_err("memshare: existing block %zu < request %u\n",
		       c->size, req->num_bytes);
		goto respond;
	}

	c->req_size = req->num_bytes;
	ms_debugfs_add(c);
	ok = true;

respond:
	resp->sequence_id_valid = 1;
	resp->sequence_id = req->sequence_id;
	if (ok) {
		resp->resp.result = QMI_RESULT_SUCCESS_V01;
		resp->resp.error = QMI_ERR_NONE_V01;
		resp->dhms_mem_alloc_addr_info_valid = 1;
		resp->dhms_mem_alloc_addr_info_len = 1;
		resp->dhms_mem_alloc_addr_info[0].phy_addr = c->dma;
		resp->dhms_mem_alloc_addr_info[0].num_bytes = req->num_bytes;
		pr_info("memshare: -> client %u gets %pad (+0x%x)\n",
			req->client_id, &c->dma, req->num_bytes);
	} else {
		resp->resp.result = QMI_RESULT_FAILURE_V01;
		resp->resp.error = QMI_ERR_NO_MEMORY_V01;
		pr_info("memshare: -> client %u allocation FAILED\n",
			req->client_id);
	}
	mutex_unlock(&ms_lock);

	ret = qmi_send_response(qmi, sq, txn, MEM_ALLOC_GENERIC_RESP_MSG_V01,
				sizeof(*resp),
				mem_alloc_generic_resp_msg_data_v01_ei, resp);
	if (ret < 0)
		pr_err("memshare: sending alloc response failed: %d\n", ret);
	kfree(resp);
}

static void ms_handle_free(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
			   struct qmi_txn *txn, const void *decoded)
{
	const struct mem_free_generic_req_msg_v01 *req = decoded;
	struct mem_free_generic_resp_msg_v01 resp = {};
	u32 i, n;
	int j, ret;

	n = min_t(u32, req->dhms_mem_alloc_addr_info_len, MAX_ARR_CNT_V01);
	pr_info("memshare: FREE %u block(s) client=%d/%u proc=%d/%u\n", n,
		req->client_id_valid, req->client_id,
		req->proc_id_valid, req->proc_id);

	mutex_lock(&ms_lock);
	for (i = 0; i < n; i++) {
		u64 addr = req->dhms_mem_alloc_addr_info[i].phy_addr;

		for (j = 0; j < MS_MAX_CLIENTS; j++) {
			if (clients[j].used && clients[j].va &&
			    clients[j].dma == addr) {
				ms_release(&clients[j]);
				break;
			}
		}
	}
	mutex_unlock(&ms_lock);

	resp.resp.result = QMI_RESULT_SUCCESS_V01;
	resp.resp.error = QMI_ERR_NONE_V01;
	ret = qmi_send_response(qmi, sq, txn, MEM_FREE_GENERIC_RESP_MSG_V01,
				MEM_FREE_REQ_MAX_MSG_LEN_V01,
				mem_free_generic_resp_msg_data_v01_ei, &resp);
	if (ret < 0)
		pr_err("memshare: sending free response failed: %d\n", ret);
}

static void ms_handle_query(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
			    struct qmi_txn *txn, const void *decoded)
{
	const struct mem_query_size_req_msg_v01 *req = decoded;
	struct mem_query_size_rsp_msg_v01 resp = {};
	int ret;

	resp.resp.result = QMI_RESULT_SUCCESS_V01;
	resp.resp.error = QMI_ERR_NONE_V01;
	resp.size_valid = 1;
	resp.size = req->client_id < ARRAY_SIZE(query_size) ?
		    query_size[req->client_id] : 0;

	pr_info("memshare: QUERY client=%u proc=%d/%u -> size 0x%x\n",
		req->client_id, req->proc_id_valid, req->proc_id, resp.size);

	ret = qmi_send_response(qmi, sq, txn, MEM_QUERY_SIZE_RESP_MSG_V01,
				MEM_QUERY_MAX_MSG_LEN_V01,
				mem_query_size_resp_msg_data_v01_ei, &resp);
	if (ret < 0)
		pr_err("memshare: sending query response failed: %d\n", ret);
}

static void ms_new_client(struct qmi_handle *qmi, unsigned int node,
			  unsigned int port)
{
	pr_info("memshare: client connected %u:%u\n", node, port);
}

static void ms_del_client(struct qmi_handle *qmi, unsigned int node,
			  unsigned int port)
{
	/* fires for every QRTR client that closes anywhere; keep quiet */
}

static const struct qmi_ops ms_ops = {
	.new_server = NULL,
	.del_client = ms_del_client,
	.net_reset = NULL,
};

static const struct qmi_msg_handler ms_handlers[] = {
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_ALLOC_GENERIC_REQ_MSG_V01,
		.ei = mem_alloc_generic_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_alloc_generic_req_msg_v01),
		.fn = ms_handle_alloc,
	},
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_FREE_GENERIC_REQ_MSG_V01,
		.ei = mem_free_generic_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_free_generic_req_msg_v01),
		.fn = ms_handle_free,
	},
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_QUERY_SIZE_REQ_MSG_V01,
		.ei = mem_query_size_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_query_size_req_msg_v01),
		.fn = ms_handle_query,
	},
	{ }
};

static int __init qcom_memshare_init(void)
{
	struct platform_device_info info = {
		.name = "qcom-memshare",
		.id = PLATFORM_DEVID_NONE,
		.dma_mask = DMA_BIT_MASK(32),
	};
	int ret;

	(void)ms_new_client;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	ms_pdev = platform_device_register_full(&info);
	if (IS_ERR(ms_pdev))
		return PTR_ERR(ms_pdev);

	ms_dbg_dir = debugfs_create_dir("memshare", NULL);

	ret = qmi_handle_init(&ms_qmi, 2048, &ms_ops, ms_handlers);
	if (ret < 0) {
		pr_err("memshare: qmi_handle_init failed: %d\n", ret);
		goto err_pdev;
	}

	ret = qmi_add_server(&ms_qmi, MEM_SHARE_SERVICE_SVC_ID,
			     MEM_SHARE_SERVICE_VERS, MEM_SHARE_SERVICE_INS_ID);
	if (ret < 0) {
		pr_err("memshare: qmi_add_server failed: %d\n", ret);
		goto err_qmi;
	}

	pr_info("memshare: service 0x%x v%d.%d registered (query sizes 0x%x 0x%x 0x%x 0x%x)\n",
		MEM_SHARE_SERVICE_SVC_ID, MEM_SHARE_SERVICE_VERS,
		MEM_SHARE_SERVICE_INS_ID, query_size[0], query_size[1],
		query_size[2], query_size[3]);
	return 0;

err_qmi:
	qmi_handle_release(&ms_qmi);
err_pdev:
	debugfs_remove_recursive(ms_dbg_dir);
	platform_device_unregister(ms_pdev);
	return ret;
}

static void __exit qcom_memshare_exit(void)
{
	int i;

	qmi_handle_release(&ms_qmi);

	mutex_lock(&ms_lock);
	for (i = 0; i < MS_MAX_CLIENTS; i++)
		if (clients[i].used)
			ms_release(&clients[i]);
	mutex_unlock(&ms_lock);

	debugfs_remove_recursive(ms_dbg_dir);
	platform_device_unregister(ms_pdev);
}

module_init(qcom_memshare_init);
module_exit(qcom_memshare_exit);

MODULE_DESCRIPTION("Qualcomm memshare QMI service (minimal mainline port)");
MODULE_LICENSE("GPL");
